#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <mutex>

using namespace Eigen;

class InverseDynamicController : public rclcpp::Node
{
public:
    InverseDynamicController()
    : Node("inverse_dynamic_controller")
    {
        using std::placeholders::_1;

        // Suscriptores y publicadores
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bebop/odom", 10, std::bind(&InverseDynamicController::odomCallback, this, _1));

        ref_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/safe_bebop/ref", 10, std::bind(&InverseDynamicController::refCallback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/safe_bebop/cmd_vel", 10);

        // Frecuencia de control 30 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&InverseDynamicController::controlLoop, this));

        // Modelo simplificado
        pPar_Model_simp_ << 0.8417, 0.18227, 0.8354, 0.17095,
                             3.966, 4.001, 9.8524, 4.7295;

        // Ganhos Dinâmicos
        Ku_ = (Vector4d() << pPar_Model_simp_(0), pPar_Model_simp_(2),
                              pPar_Model_simp_(4), pPar_Model_simp_(6)).finished().asDiagonal();
        Kv_ = (Vector4d() << pPar_Model_simp_(1), pPar_Model_simp_(3),
                              pPar_Model_simp_(5), pPar_Model_simp_(7)).finished().asDiagonal();

        // Ganhos Controlador
        VectorXd gains(16);
        gains << 2, 2, 3, 1.5,
                 2, 2, 1.8, 5,
                 1, 1, 1, 1.5,
                 1, 1, 1, 1;

        Ksp_ = gains.segment(0,4).asDiagonal();
        Ksd_ = gains.segment(4,4).asDiagonal();
        Kp_  = gains.segment(8,4).asDiagonal();
        Kd_  = gains.segment(12,4).asDiagonal();

        // Inicialización de variables
        X_.setZero(); dX_.setZero(); Xd_.setZero(); dXd_.setZero(); ddXd_.setZero();
        Xtil_.setZero(); dXtil_.setZero();
        Ucw_.setZero(); Ucw_ant_.setZero();
        Ud_.setZero();
        t_last_ = this->now();
        kinematics_control_ = 0;

        RCLCPP_INFO(this->get_logger(), "Controlador Inverso Dinámico listo.");
    }

private:
    // ROS infra
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time t_last_;
    std::mutex mtx_;

    // Parámetros y estados
    VectorXd pPar_Model_simp_{8};
    Matrix4d Ku_, Kv_, Ksp_, Ksd_, Kp_, Kd_;
    Vector4d X_, dX_, Xd_, dXd_, ddXd_, Xtil_, dXtil_, Ucw_, Ucw_ant_, Ud_;
    int kinematics_control_;

    // --- CALLBACKS ---
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::scoped_lock lock(mtx_);

        double psi = tf2::getYaw(msg->pose.pose.orientation);
        X_ << msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z,
              psi;

        // Velocidades del cuerpo → transformar a mundo
        Vector3d v_body(msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);

        Matrix3d R;
        R << cos(psi), -sin(psi), 0,
             sin(psi),  cos(psi), 0,
             0,         0,        1;

        Vector3d v_world = R * v_body;
        dX_ << v_world(0), v_world(1), v_world(2),
               msg->twist.twist.angular.z;
    }

    void refCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::scoped_lock lock(mtx_);
        if (msg->data.size() < 8) return;
        Xd_  << msg->data[0], msg->data[1], msg->data[2], msg->data[3];
        dXd_ << msg->data[4], msg->data[5], msg->data[6], msg->data[7];
        ddXd_.setZero();
    }

    // --- LOOP PRINCIPAL DE CONTROL ---
    void controlLoop()
    {
        std::scoped_lock lock(mtx_);
        double dt = (this->now() - t_last_).seconds();
        if (dt <= 0.0) dt = 0.033;
        t_last_ = this->now();

        // X, dX, Xd, dXd, ddXd ya actualizados en callbacks

        // === Errores ===
        if (Xtil_.norm() == 0.0) {
            Xtil_  = Xd_  - X_;
            dXtil_ = dXd_ - dX_;
        } else {
            // Mantiene forma del MATLAB, aunque aquí ya están almacenados
            Xtil_  = Xd_  - X_;
            dXtil_ = dXd_ - dX_;
        }

        if (std::abs(Xtil_(3)) > M_PI) {
            if (Xtil_(3) > 0)
                Xtil_(3) = -2 * M_PI + Xtil_(3);
            else
                Xtil_(3) =  2 * M_PI + Xtil_(3);
        }

        // === Controle cinemático ===
        Ucw_ant_ = Ucw_;
        Ucw_ = dXd_ + Ksp_ * (Kp_ * Xtil_).array().tanh().matrix();

        if (kinematics_control_ == 1)
            Ucw_.head(3) = dX_.head(3);

        Vector4d dUcw = (Ucw_ - Ucw_ant_) / dt;

        // === Cinemática directa ===
        double psi = X_(3);
        Matrix4d F;
        F << cos(psi), -sin(psi), 0, 0,
             sin(psi),  cos(psi), 0, 0,
             0,         0,        1, 0,
             0,         0,        0, 1;

        // === Compensador dinámico ===
        Vector4d Udw = (F * Ku_).inverse() * (dUcw + Ksd_ * (Ucw_ - dX_) + Kv_ * dX_);

        // === Comandos enviados al Bebop 2 ===
        Ud_(0) = Udw(0);
        Ud_(1) = Udw(1);
        Ud_(2) = Udw(2);
        Ud_(3) = 0;
        // 4,5 no rotan
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = Ud_(0);
        cmd.linear.y  = Ud_(1);
        cmd.linear.z  = Ud_(2);
        cmd.angular.z = Udw(3);
        cmd_pub_->publish(cmd);
    }
};

// --- MAIN ---
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InverseDynamicController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
