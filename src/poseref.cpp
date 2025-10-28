#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <deque>

using namespace std::chrono_literals;

class TagRefPublisher : public rclcpp::Node {
public:
  TagRefPublisher()
  : Node("tag_ref_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tag_detected_(false)
  {
    ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose/ref", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ref_marker", 10);

    sub_detected_ = this->create_subscription<std_msgs::msg::Bool>(
      "/bebop/detected", 10,
      std::bind(&TagRefPublisher::tagDetectedCallback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/bebop/odom", 10,
      std::bind(&TagRefPublisher::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(50ms, std::bind(&TagRefPublisher::updateLoop, this));

    RCLCPP_INFO(this->get_logger(), "TagRefPublisher with smoothing (N=10) running");
  }

private:
  // --- Callbacks ---
  void tagDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    tag_detected_ = msg->data;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_pose_ = msg->pose.pose;
  }

  // --- Main update loop ---
  void updateLoop() {
    if (!tag_detected_) return;

    geometry_msgs::msg::TransformStamped tf_tag;
    try {
      tf_tag = tf_buffer_.lookupTransform("odom", "tag_0", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed: %s", ex.what());
      return;
    }

    tf2::Transform T_tag;
    tf2::fromMsg(tf_tag.transform, T_tag);

    // --- Desired offset: 1.5 m front (z_tag), +0.3 m up (z_odom) ---
    tf2::Vector3 offset_in_tag(0.0, 0.0, 1.5);
    tf2::Vector3 p_ref_in_odom = T_tag * offset_in_tag;
    p_ref_in_odom.setZ(p_ref_in_odom.z() + 0.30);

    // --- Add to buffer for smoothing ---
    addToBuffer(p_ref_in_odom);

    // Compute averaged position
    tf2::Vector3 p_avg = getAverage();

    // --- PoseStamped ---
    geometry_msgs::msg::PoseStamped ref_msg;
    ref_msg.header.stamp = this->now();
    ref_msg.header.frame_id = "odom";
    ref_msg.pose.position.x = p_avg.x();
    ref_msg.pose.position.y = p_avg.y();
    ref_msg.pose.position.z = p_avg.z();
    ref_msg.pose.orientation = tf_tag.transform.rotation;
    ref_pub_->publish(ref_msg);

    // --- Marker visualization ---
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "tag_ref";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = ref_msg.pose;
    marker.scale.x = 0.066; // 1/3 del tamaño anterior (0.2 / 3)
    marker.scale.y = 0.066;
    marker.scale.z = 0.066;

    marker.color.a = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.9;
    marker.color.b = 0.2;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker_pub_->publish(marker);
  }

  // --- Smoothing helpers ---
  void addToBuffer(const tf2::Vector3 &point) {
    buffer_.push_back(point);
    if (buffer_.size() > window_size_) buffer_.pop_front();
  }

  tf2::Vector3 getAverage() const {
    if (buffer_.empty()) return tf2::Vector3(0,0,0);
    tf2::Vector3 sum(0,0,0);
    for (const auto &p : buffer_) sum += p;
    return sum / static_cast<double>(buffer_.size());
  }

  // --- ROS interfaces ---
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_detected_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::Pose last_odom_pose_;
  bool tag_detected_;

  // --- Filtering ---
  std::deque<tf2::Vector3> buffer_;
  const size_t window_size_ = 10;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagRefPublisher>());
  rclcpp::shutdown();
  return 0;
}
