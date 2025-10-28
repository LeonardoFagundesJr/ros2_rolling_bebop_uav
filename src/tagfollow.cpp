// DroneForceVisualizerXYZ: ROS2 node that subscribes to /bebop/camera_moving and /bebop/detected,
// computes 3D forces (Fx, Fy, Fz) using a PID controller from the transform base_link→tag_0,
// visualizes them as an arrow marker in RViz, and publishes the same forces as velocity commands
// on /bebop/cmd_vel, limited to ±1.0 m/s at 20 Hz.
// Author: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar), in collaboration with UFV.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
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
    tag_detected_(false),
    iter_count_(0)
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("drone_force_marker", 10);
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);

    sub_camera_moving_ = this->create_subscription<std_msgs::msg::Bool>(
      "/bebop/camera_moving", 10,
      std::bind(&DroneForceVisualizerXYZ::cameraMovingCallback, this, std::placeholders::_1));
    sub_tag_detected_ = this->create_subscription<std_msgs::msg::Bool>(
      "/bebop/detected", 10,
      std::bind(&DroneForceVisualizerXYZ::tagDetectedCallback, this, std::placeholders::_1));

    // Parámetros PID
    this->declare_parameter("k_x", 0.0);
    this->declare_parameter("k_y", 1.4);
    this->declare_parameter("k_z", 0.0);
    this->declare_parameter("kd_x", 0.00);
    this->declare_parameter("kd_y", 0.20);
    this->declare_parameter("kd_z", 0.00);
    this->declare_parameter("ki_x", 0.000000);
    this->declare_parameter("ki_y", 0.2);
    this->declare_parameter("ki_z", 0.000000);
    this->declare_parameter("desired_distance", 2.5);
    this->declare_parameter("z_ref_offset", 0.3);
    this->declare_parameter("force_scale", 1.0);
    this->declare_parameter("max_vel", 1.0);

    this->get_parameter("k_x", kx_);
    this->get_parameter("k_y", ky_);
    this->get_parameter("k_z", kz_);
    this->get_parameter("kd_x", kdx_);
    this->get_parameter("kd_y", kdy_);
    this->get_parameter("kd_z", kdz_);
    this->get_parameter("ki_x", kix_);
    this->get_parameter("ki_y", kiy_);
    this->get_parameter("ki_z", kiz_);
    this->get_parameter("desired_distance", d_ref_);
    this->get_parameter("z_ref_offset", z_ref_);
    this->get_parameter("force_scale", scale_);
    this->get_parameter("max_vel", max_vel_);

    dt_ = 0.05;  // 20 Hz
    ex_prev_ = ey_prev_ = ez_prev_ = 0.0;
    ix_ = iy_ = iz_ = 0.0;

    // Control loop at 20 Hz
    timer_ = this->create_wall_timer(50ms, std::bind(&DroneForceVisualizerXYZ::updateLoop, this));
  }

private:
  void cameraMovingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    camera_moving_ = msg->data;
  }

  void tagDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    tag_detected_ = msg->data;
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

      // === Integrador cada 4 iteraciones (5 Hz) ===
      iter_count_++;
      if (iter_count_ % 4 == 0) {
        double dt_int = dt_ * 4.0;  // periodo efectivo de integración (0.2 s)
        ix_ += ex * dt_int;
        iy_ += ey * dt_int;
        iz_ += ez * dt_int;

        const double i_limit = max_vel_ / std::max({kix_, kiy_, kiz_, 1e-6});
        ix_ = std::clamp(ix_, -i_limit, i_limit);
        iy_ = std::clamp(iy_, -i_limit, i_limit);
        iz_ = std::clamp(iz_, -i_limit, i_limit);
      }

      // === Control PID ===
      Fx = kx_ * ex + kdx_ * dex + kix_ * ix_;
      Fy = ky_ * ey + kdy_ * dey + kiy_ * iy_;
      Fz = kz_ * ez + kdz_ * dez + kiz_ * iz_;

      ex_prev_ = ex;
      ey_prev_ = ey;
      ez_prev_ = ez;

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
    arrow.pose.position.x = 0.0;
    arrow.pose.position.y = 0.0;
    arrow.pose.position.z = 0.0;

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

  void resetIntegrators() {
    ix_ = iy_ = iz_ = 0.0;
    iter_count_ = 0;
  }

  // ROS interfaces
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_camera_moving_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_tag_detected_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parámetros del controlador
  double kx_, ky_, kz_;
  double kdx_, kdy_, kdz_;
  double kix_, kiy_, kiz_;
  double d_ref_, z_ref_, scale_, max_vel_;
  double ex_prev_, ey_prev_, ez_prev_, dt_;
  double ix_, iy_, iz_;
  int iter_count_;  // contador para integrador

  // Estado
  bool camera_moving_;
  bool tag_detected_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneForceVisualizerXYZ>());
  rclcpp::shutdown();
  return 0;
}
