#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

class TagFollower : public rclcpp::Node
{
public:
  TagFollower()
  : Node("tagfollow"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Publicador de velocidades
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);

    // Timer de control (50 Hz)
    timer_ = this->create_wall_timer(
      20ms, std::bind(&TagFollower::controlLoop, this));

    // Parámetros ajustables
    this->declare_parameter("k_x", 0.5);
    this->declare_parameter("k_y", 0.5);
    this->declare_parameter("k_z", 0.8);
    this->declare_parameter("desired_distance", 1.0);
    this->declare_parameter("min_altitude", 0.4);

    this->get_parameter("k_x", kx_);
    this->get_parameter("k_y", ky_);
    this->get_parameter("k_z", kz_);
    this->get_parameter("desired_distance", d_ref_);
    this->get_parameter("min_altitude", min_alt_);

    RCLCPP_INFO(this->get_logger(),
      "Nodo tagfollow iniciado (usa TF: camera_gimbal -> tag_0, altura de seguridad %.2f m, distancia deseada %.2f m)",
      min_alt_, d_ref_);
  }

private:
  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    try
    {
      // 1. Obtener la posición del tag respecto a la cámara
      geometry_msgs::msg::TransformStamped tf_tag =
        tf_buffer_.lookupTransform("camera_gimbal", "tag_0", tf2::TimePointZero);

      double x = tf_tag.transform.translation.x;  // lateral
      double y = tf_tag.transform.translation.y;  // vertical
      double z = tf_tag.transform.translation.z;  // distancia frontal

      // 2. Obtener altura del dron sobre el plano odom
      geometry_msgs::msg::TransformStamped tf_base =
        tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
      double drone_alt = tf_base.transform.translation.z;

      // 3. Si la altura es baja, detener por seguridad
      if (drone_alt < min_alt_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Altura baja (%.2f m sobre odom): deteniendo el dron", drone_alt);
        cmd_pub_->publish(cmd);
        return;
      }

      // 4. Control proporcional: seguir el tag
      cmd.linear.x = -kx_ * (z - d_ref_); // acercarse/alejarse
      cmd.linear.y = -ky_ * x;            // centrar horizontalmente
      cmd.linear.z = -kz_ * y;            // igualar altura del tag
      cmd.angular.z = 0.0;                // sin control de yaw

      cmd_pub_->publish(cmd);
    }
    catch (tf2::TransformException &ex)
    {
      // Si no se ve el tag o no hay TF válido
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Transform no encontrada (camera_gimbal->tag_0 o odom->base_link): %s", ex.what());

      // Enviar velocidades nulas (hover)
      cmd_pub_->publish(cmd);
    }
  }

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parámetros
  double kx_, ky_, kz_, d_ref_, min_alt_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagFollower>());
  rclcpp::shutdown();
  return 0;
}
