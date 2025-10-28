// InverseDynamicControllerROS2
// Autor: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar), 2025
// Controlador acoplado cinemático + dinámico del Bebop2, ejecutado a 50 Hz.
// Usa /bebop/odom como feedback, /pose/ref como referencia, y detiene el control si /bebop/detected = true.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>
#include <cmath>

using namespace std::chrono_literals;

class InverseDynamicController : public rclcpp::Node {
public:
  InverseDynamicController()
  : Node("inverse_dynamic_controller"), tag_detected_(false)
  {
    // ---- Publishers & Subscribers ----
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);

    sub_ref_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose/ref", 10,
      std::bind(&InverseDynamicController::refCallback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/bebop/odom", 10,
      std::bind(&InverseDynamicController::odomCallback, this, std::placeholders::_1));

    sub_tag_detected_ = this->create_subscription<std_msgs::msg::Bool>(
      "/bebop/detected", 10,
      std::bind(&InverseDynamicController::tagDetectedCallback, this, std::placeholders::_1));

    // ---- Modelo simplificado ----
    model_simp_ << 0.8417, 0.18227, 0.8354, 0.17095, 3.966, 4.001, 9.8524, 4.7295;
    Ku_ = Eigen::DiagonalMatrix<double, 4>(model_simp_(0), model_simp_(2), model_simp_(4), model_simp_(6));
    Kv_ = Eigen::DiagonalMatrix<double, 4>(model_simp_(1), model_simp_(3), model_simp_(5), model_simp_(7));

    // ---- Ganhos [X Y Z Psi] ----
    gains_ << 2,2,3,1.5, 2,2,1.8,5, 1,1,1,1.5, 1,1,1,1;
    setGains();

    // ---- Estados iniciales ----
    X_.setZero(); dX_.setZero();
    Xd_.setZero(); dXd_.setZero(); ddXd_.setZero();
    Ucw_.setZero(); Ucw_ant_.setZero();

    dt_ = 0.02;  // 50 Hz
    timer_ = this->create_wall_timer(20ms, std::bind(&InverseDynamicController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Inverse Dynamic Controller running at 50 Hz");
  }

private:
  // ==================== CALLBACKS ====================

  void refCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    Xd_(0) = msg->pose.position.x;
    Xd_(1) = msg->pose.position.y;
    Xd_(2) = msg->pose.position.z;

    // yaw desde quaternion
    auto q = msg->pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    Xd_(3) = std::atan2(siny_cosp, cosy_cosp);

    // La referencia de velocidad es cero (mantener posición)
    dXd_.setZero();
    ddXd_.setZero();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Posición actual
    X_(0) = msg->pose.pose.position.x;
    X_(1) = msg->pose.pose.position.y;
    X_(2) = msg->pose.pose.position.z;

    // yaw actual
    auto q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    X_(3) = std::atan2(siny_cosp, cosy_cosp);

    // Velocidades lineales y angular yaw
    dX_(0) = msg->twist.twist.linear.x;
    dX_(1) = msg->twist.twist.linear.y;
    dX_(2) = msg->twist.twist.linear.z;
    dX_(3) = msg->twist.twist.angular.z;
  }

  void tagDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    tag_detected_ = msg->data;
  }

  // ==================== CONTROL PRINCIPAL ====================

  void controlLoop() {
    if (!tag_detected_) {
      publishZero();
      return;
    }

    // Error
    Eigen::Vector4d Xtil = Xd_ - X_;
    Eigen::Vector4d dXtil = dXd_ - dX_;

    // Corrige discontinuidad en yaw
    if (std::fabs(Xtil(3)) > M_PI) {
      if (Xtil(3) > 0) Xtil(3) -= 2*M_PI;
      else Xtil(3) += 2*M_PI;
    }

    // --- Control cinemático ---
    Ucw_ = dXd_ + Ksp_ * ( (Kp_ * Xtil).array().tanh() ).matrix();
    Eigen::Vector4d dUcw = (Ucw_ - Ucw_ant_) / dt_;
    Ucw_ant_ = Ucw_;

    // --- Matriz cinemática F ---
    double psi = X_(3);
    Eigen::Matrix4d F;
    F << cos(psi), -sin(psi), 0, 0,
         sin(psi),  cos(psi), 0, 0,
         0,         0,        1, 0,
         0,         0,        0, 1;

    // --- Compensador dinámico ---
    Eigen::Vector4d Udw = (F * Ku_).inverse() * (dUcw + Ksd_ * (Ucw_ - dX_) + Kv_ * dX_);
    for (int i = 0; i < 4; ++i) {
        if (Udw(i) > 1.0)  Udw(i) = 1.0;
        if (Udw(i) < -1.0) Udw(i) = -1.0;
        }

    // --- Publicación de velocidades ---
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = Udw(0);
    cmd.linear.y  = Udw(1);
    cmd.linear.z  = Udw(2);
    cmd.angular.z = Udw(3);

    cmd_pub_->publish(cmd);
  }

  void publishZero() {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = stop.linear.y = stop.linear.z = stop.angular.z = 0.0;
    cmd_pub_->publish(stop);
  }

  // ==================== CONFIGURACIÓN DE GANANCIAS ====================

  void setGains() {
    Ksp_.setZero(); Ksd_.setZero(); Kp_.setZero(); Kd_.setZero();
    for (int i = 0; i < 4; ++i) {
      Ksp_(i,i) = gains_(i);
      Ksd_(i,i) = gains_(i+4);
      Kp_(i,i)  = gains_(i+8);
      Kd_(i,i)  = gains_(i+12);
    }
  }

  // ==================== VARIABLES ====================

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ref_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_tag_detected_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Ganancias y modelo
  Eigen::VectorXd model_simp_{8};
  Eigen::DiagonalMatrix<double,4> Ku_, Kv_;
  Eigen::Matrix4d Ksp_, Ksd_, Kp_, Kd_;
  Eigen::VectorXd gains_{16};

  // Estados
  Eigen::Vector4d X_, dX_;
  Eigen::Vector4d Xd_, dXd_, ddXd_;
  Eigen::Vector4d Ucw_, Ucw_ant_;

  double dt_;
  bool tag_detected_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseDynamicController>());
  rclcpp::shutdown();
  return 0;
}
