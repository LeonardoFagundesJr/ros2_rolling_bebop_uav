#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <cmath>

using namespace std::chrono_literals;

class DroneForceVisualizerXYZ : public rclcpp::Node
{
public:
  DroneForceVisualizerXYZ()
  : Node("drone_force_visualizer_xyz"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "drone_force_marker", 10);

    // Parámetros
    this->declare_parameter("k_x", 0.2);
    this->declare_parameter("k_y", 0.2);
    this->declare_parameter("k_z", 0.2);
    this->declare_parameter("desired_distance", 2.0);  // 2 m frente al tag
    this->declare_parameter("z_ref_offset", 0.3);       // 30 cm por encima
    this->declare_parameter("force_scale", 1.0);

    this->get_parameter("k_x", kx_);
    this->get_parameter("k_y", ky_);
    this->get_parameter("k_z", kz_);
    this->get_parameter("desired_distance", d_ref_);
    this->get_parameter("z_ref_offset", z_ref_);
    this->get_parameter("force_scale", scale_);

    timer_ = this->create_wall_timer(100ms, std::bind(&DroneForceVisualizerXYZ::updateMarker, this));

    RCLCPP_INFO(get_logger(),
      "Visualizador de fuerzas XYZ iniciado (usa base_link -> tag_0, dist=%.2f m, offset Z=%.2f m)",
      d_ref_, z_ref_);
  }

private:
  void updateMarker()
  {
    double Fx = 0.0, Fy = 0.0, Fz = 0.0;

    try
    {
      geometry_msgs::msg::TransformStamped tf_tag =
        tf_buffer_.lookupTransform("base_link", "tag_0", tf2::TimePointZero);

      double x = tf_tag.transform.translation.x; // frente
      double y = tf_tag.transform.translation.y; // lateral
      double z = tf_tag.transform.translation.z; // vertical

      // Control proporcional 3D
      Fx =  kx_ * (x - d_ref_);
      Fy =  ky_ * y;
      Fz =  kz_ * (z + z_ref_);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Tag pos [x=%.2f, y=%.2f, z=%.2f] -> F[%.2f, %.2f, %.2f]",
        x, y, z, Fx, Fy, Fz);
    }
    catch (tf2::TransformException &ex)
    {
      // Tag perdido → fuerza cero inmediata
      Fx = Fy = Fz = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Transform no encontrada (base_link->tag_0): fuerza = 0.0 (%s)", ex.what());
    }

    // Publicar siempre (incluso si el tag se perdió)
    publishForceArrow(Fx, Fy, Fz);
  }

  void publishForceArrow(double Fx, double Fy, double Fz)
  {
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

    // Magnitud
    double magnitude = std::sqrt(Fx*Fx + Fy*Fy + Fz*Fz) * scale_;
    arrow.scale.x = magnitude;
    arrow.scale.y = 0.04;
    arrow.scale.z = 0.04;

    // Si la fuerza es cero, esconder la flecha (sin orientación)
    if (magnitude < 1e-6) {
      arrow.color.a = 0.0;
      marker_pub_->publish(arrow);
      return;
    }

    // Orientación
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

    // Color: amarillo
    arrow.color.a = 1.0;
    arrow.color.r = 1.0;
    arrow.color.g = 0.9;
    arrow.color.b = 0.1;

    marker_pub_->publish(arrow);
  }

  // ROS
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  double kx_, ky_, kz_, d_ref_, z_ref_, scale_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneForceVisualizerXYZ>());
  rclcpp::shutdown();
  return 0;
}
