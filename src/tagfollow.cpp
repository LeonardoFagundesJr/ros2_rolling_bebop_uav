// DroneForceVisualizerXYZ: PID con Kp variable mediante tangente hiperbólica
// Autor: Brayan Saldarriaga-Mesa (UFV collaboration)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class DroneForceVisualizerXYZ : public rclcpp::Node {
public:
  DroneForceVisualizerXYZ()
  : Node("drone_force_visualizer_xyz"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    camera_moving_(false),
    tag_detected_(false)
  {
    // --- Publishers ---
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("drone_force_marker", 2);
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/safe_bebop/cmd_vel", 2);

    // --- Subscribers ---
    sub_camera_moving_ = this->create_subscription<std_msgs::msg::Bool>(
      "/bebop/camera_moving", 2,
      std::bind(&DroneForceVisualizerXYZ::cameraMovingCallback, this, std::placeholders::_1));

    sub_tag_detected_ = this->create_subscription<std_msgs::msg::Bool>(
      "/bebop/detected", 2,
      std::bind(&DroneForceVisualizerXYZ::tagDetectedCallback, this, std::placeholders::_1));

    sub_pid_gains_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/drone/pid_gains", 1,
      std::bind(&DroneForceVisualizerXYZ::pidGainsCallback, this, std::placeholders::_1));

    // --- Parámetros ---
    this->declare_parameter("desired_distance", 2.0);
    this->declare_parameter("z_ref_offset", 0.3);
    this->declare_parameter("force_scale", 3.0);
    this->declare_parameter("max_vel", 1.0);
    this->declare_parameter("transition_error", 0.5);
    this->declare_parameter("alpha", 6.0);

    this->get_parameter("desired_distance", d_ref_);
    this->get_parameter("z_ref_offset", z_ref_);
    this->get_parameter("force_scale", scale_);
    this->get_parameter("max_vel", max_vel_);
    this->get_parameter("transition_error", e_transition_);
    this->get_parameter("alpha", alpha_);

    // --- Inicialización PID ---
    Kp1x_ = Kp1y_ = Kp1z_ = 0.0;
    Kp2x_ = Kp2y_ = Kp2z_ = 0.0;
    kdx_  = kdy_  = kdz_  = 0.0;
    kix_  = kiy_  = kiz_  = 0.0;

    dt_ = 1.0 / 30.0;  // 30 Hz
    ex_prev_ = ey_prev_ = ez_prev_ = 0.0;
    ix_ = iy_ = iz_ = 0.0;

    timer_ = this->create_wall_timer(33ms, std::bind(&DroneForceVisualizerXYZ::updateLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "DroneForceVisualizerXYZ iniciado: Kp tanh(alpha=%.1f), salida /safe_bebop/cmd_vel @30 Hz", alpha_);
  }

  void publishZeroOutputs() {
    geometry_msgs::msg::Twist zero_cmd;
    zero_cmd.linear.x = zero_cmd.linear.y = zero_cmd.linear.z = zero_cmd.angular.z = 0.0;
    cmd_pub_->publish(zero_cmd);

    visualization_msgs::msg::Marker zero_marker;
    zero_marker.header.frame_id = "base_link";
    zero_marker.header.stamp = now();
    zero_marker.ns = "drone_force_xyz";
    zero_marker.id = 1;
    zero_marker.type = visualization_msgs::msg::Marker::ARROW;
    zero_marker.action = visualization_msgs::msg::Marker::ADD;
    zero_marker.scale.x = 0.0;
    zero_marker.scale.y = 0.4;
    zero_marker.scale.z = 0.0;
    zero_marker.color.a = 0.0;
    marker_pub_->publish(zero_marker);
  }

private:
  // --- Callbacks de suscriptores ---
  void cameraMovingCallback(const std_msgs::msg::Bool::SharedPtr msg) { camera_moving_ = msg->data; }
  void tagDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg) { tag_detected_ = msg->data; }

  // Recibe [Kp1x,Kp1y,Kp1z, Kp2x,Kp2y,Kp2z, Kdx,Kdy,Kdz, Kix,Kiy,Kiz]
  void pidGainsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 12) {
      RCLCPP_WARN(this->get_logger(), "PID gains message must have 12 values");
      return;
    }

    Kp1x_ = msg->data[0]; Kp1y_ = msg->data[1]; Kp1z_ = msg->data[2];
    Kp2x_ = msg->data[3]; Kp2y_ = msg->data[4]; Kp2z_ = msg->data[5];
    kdx_  = msg->data[6]; kdy_  = msg->data[7]; kdz_  = msg->data[8];
    kix_  = msg->data[9]; kiy_  = msg->data[10]; kiz_ = msg->data[11];

    RCLCPP_INFO(this->get_logger(),
      "PID updated: Kp1=(%.2f,%.2f,%.2f)  Kp2=(%.2f,%.2f,%.2f)  Kd=(%.2f,%.2f,%.2f)  Ki=(%.2f,%.2f,%.2f)",
      Kp1x_, Kp1y_, Kp1z_, Kp2x_, Kp2y_, Kp2z_, kdx_, kdy_, kdz_, kix_, kiy_, kiz_);
  }

  // --- Función de ganancia variable con tanh ---
  double variableGain(double error, double Kp_low, double Kp_high) {
    double e = std::abs(error);
    double blend = 0.3 * (1.0 - std::tanh(alpha_ * (e - e_transition_)));
    return Kp_low + (Kp_high - Kp_low) * blend;
  }

  void updateLoop() {
    double Fx = 0.0, Fy = 0.0, Fz = 0.0;

    if (camera_moving_ || !tag_detected_) {
      resetIntegrators();
      publishVelocity(0.0, 0.0, 0.0);
      publishForceArrow(0.0, 0.0, 0.0);
      return;
    }

    try {
      geometry_msgs::msg::TransformStamped tf_tag =
        tf_buffer_.lookupTransform("base_link", "tag_0", tf2::TimePointZero);

      double x = tf_tag.transform.translation.x;
      double y = tf_tag.transform.translation.y;
      double z = tf_tag.transform.translation.z;

      double ex = x - d_ref_;
      double ey = y;
      double ez = z + z_ref_;

      double dex = (ex - ex_prev_) / dt_;
      double dey = (ey - ey_prev_) / dt_;
      double dez = (ez - ez_prev_) / dt_;

      // --- Ganancia proporcional variable (tanh) ---
      double Kpx = variableGain(ex, Kp1x_, Kp2x_);
      double Kpy = variableGain(ey, Kp1y_, Kp2y_);
      double Kpz = variableGain(ez, Kp1z_, Kp2z_);

      Fx = Kpx * ex;
      Fy = Kpy * ey;
      Fz = Kpz * ez;

      if (std::abs(ex) < 0.3) {
        dex = (ex - ex_prev_) / dt_;
        Fx += kdx_ * dex;
      } else {
        dex = 0.0;
        ex_prev_ = ex;  // reinicia referencia
      }

      if (std::abs(ey) < 0.3) {
        dey = (ey - ey_prev_) / dt_;
        Fy += kdy_ * dey;
      } else {
        dey = 0.0;
        ey_prev_ = ey;
      }

      if (std::abs(ez) < 0.3) {
        dez = (ez - ez_prev_) / dt_;
        Fz += kdz_ * dez;
      } else {
        dez = 0.0;
        ez_prev_ = ez;
      }

      // --- Integrativo: solo en ±0.1 m ---
      if (std::abs(ex) < 0.1) ix_ += ex * dt_;
      else ix_ = 0.0;

      if (std::abs(ey) < 0.1) iy_ += ey * dt_;
      else iy_ = 0.0;

      if (std::abs(ez) < 0.1) iz_ += ez * dt_;
      else iz_ = 0.0;

      Fx += kix_ * ix_;
      Fy += kiy_ * iy_;
      Fz += kiz_ * iz_;

      ex_prev_ = ex; ey_prev_ = ey; ez_prev_ = ez;
    } catch (tf2::TransformException &ex) {
      Fx = Fy = Fz = 0.0;
    }

    Fx = std::clamp(Fx, -max_vel_, max_vel_);
    Fy = std::clamp(Fy, -max_vel_, max_vel_);
    Fz = std::clamp(Fz, -max_vel_, max_vel_);

    publishVelocity(Fx, Fy, Fz);
    publishForceArrow(Fx, Fy, Fz);
  }

  void publishVelocity(double Fx, double Fy, double Fz) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = Fx;
    cmd.linear.y = Fy;
    cmd.linear.z = Fz;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void publishForceArrow(double Fx, double Fy, double Fz) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "base_link";
    arrow.header.stamp = now();
    arrow.ns = "drone_force_xyz";
    arrow.id = 1;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.position.x = arrow.pose.position.y = arrow.pose.position.z = 0.0;

    double magnitude = std::sqrt(Fx * Fx + Fy * Fy + Fz * Fz) * scale_;
    arrow.scale.x = magnitude;
    arrow.scale.y = 0.04;
    arrow.scale.z = 0.04;

    if (magnitude < 1e-6) {
      arrow.color.a = 0.0;
      marker_pub_->publish(arrow);
      return;
    }

    tf2::Vector3 from(1, 0, 0);
    tf2::Vector3 to(Fx, Fy, Fz);
    to.normalize();
    tf2::Quaternion q;
    q.setRotation(from.cross(to), std::acos(from.dot(to)));
    q.normalize();

    arrow.pose.orientation.x = q.x();
    arrow.pose.orientation.y = q.y();
    arrow.pose.orientation.z = q.z();
    arrow.pose.orientation.w = q.w();

    arrow.color.a = 1.0;
    arrow.color.r = 1.0;
    arrow.color.g = 0.9;
    arrow.color.b = 0.1;
    marker_pub_->publish(arrow);
  }

  void resetIntegrators() { ix_ = iy_ = iz_ = 0.0; }

  // --- ROS interfaces ---
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_camera_moving_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_tag_detected_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pid_gains_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- PID parámetros ---
  double Kp1x_, Kp1y_, Kp1z_;
  double Kp2x_, Kp2y_, Kp2z_;
  double kdx_, kdy_, kdz_;
  double kix_, kiy_, kiz_;
  double d_ref_, z_ref_, scale_, max_vel_;
  double e_transition_, alpha_;
  double ex_prev_, ey_prev_, ez_prev_, dt_;
  double ix_, iy_, iz_;
  bool camera_moving_, tag_detected_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneForceVisualizerXYZ>();

  try { rclcpp::spin(node); }
  catch (const std::exception &e) { RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what()); }
  catch (...) { RCLCPP_ERROR(node->get_logger(), "Unknown exception caught."); }

  node->publishZeroOutputs();
  rclcpp::shutdown();
  return 0;
}
