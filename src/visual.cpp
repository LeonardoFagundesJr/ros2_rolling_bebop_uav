// DroneForceVisualizerXYZ: ROS2 node that subscribes to /bebop/camera_moving and /bebop/detected,
// computes proportional 3D forces (Fx, Fy, Fz) from the transform base_link→tag_0, 
// and visualizes them as an arrow marker in RViz.
// Author: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar), in collaboration with UFV.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <tf2/LinearMath/Vector3.h>
#include <cmath>

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
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("drone_force_marker", 10);
    sub_camera_moving_ = this->create_subscription<std_msgs::msg::Bool>(
      "/bebop/camera_moving", 10,
      std::bind(&DroneForceVisualizerXYZ::cameraMovingCallback, this, std::placeholders::_1));
    sub_tag_detected_ = this->create_subscription<std_msgs::msg::Bool>(
      "/bebop/detected", 10,
      std::bind(&DroneForceVisualizerXYZ::tagDetectedCallback, this, std::placeholders::_1));

    this->declare_parameter("k_x", 0.2);
    this->declare_parameter("k_y", 0.2);
    this->declare_parameter("k_z", 0.2);
    this->declare_parameter("desired_distance", 2.0);
    this->declare_parameter("z_ref_offset", 0.3);
    this->declare_parameter("force_scale", 1.0);

    this->get_parameter("k_x", kx_);
    this->get_parameter("k_y", ky_);
    this->get_parameter("k_z", kz_);
    this->get_parameter("desired_distance", d_ref_);
    this->get_parameter("z_ref_offset", z_ref_);
    this->get_parameter("force_scale", scale_);

    timer_ = this->create_wall_timer(100ms, std::bind(&DroneForceVisualizerXYZ::updateMarker, this));
  }

private:
  void cameraMovingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    camera_moving_ = msg->data;
  }

  void tagDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    tag_detected_ = msg->data;
  }

  void updateMarker() {
    double Fx = 0.0, Fy = 0.0, Fz = 0.0;
    if (camera_moving_ || !tag_detected_) {
      publishForceArrow(0.0, 0.0, 0.0);
      return;
    }

    try {
      geometry_msgs::msg::TransformStamped tf_tag =
        tf_buffer_.lookupTransform("base_link", "tag_0", tf2::TimePointZero);

      double x = tf_tag.transform.translation.x;
      double y = tf_tag.transform.translation.y;
      double z = tf_tag.transform.translation.z;

      Fx =  kx_ * (x - d_ref_);
      Fy =  ky_ * y;
      Fz =  kz_ * (z + z_ref_);
    } catch (tf2::TransformException &ex) {
      Fx = Fy = Fz = 0.0;
    }

    publishForceArrow(Fx, Fy, Fz);
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

    double magnitude = std::sqrt(Fx*Fx + Fy*Fy + Fz*Fz) * scale_;
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

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_camera_moving_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_tag_detected_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  double kx_, ky_, kz_, d_ref_, z_ref_, scale_;
  bool camera_moving_;
  bool tag_detected_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneForceVisualizerXYZ>());
  rclcpp::shutdown();
  return 0;
}
