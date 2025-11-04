// BebopJoyNode: ROS2 node that maps joystick inputs to Bebop drone control commands,
// publishing velocity (cmd_vel), takeoff, and landing messages based on joystick axes and button states.
// Author: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar), in collaboration with UFV.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>

class BebopJoyNode : public rclcpp::Node {
public:
  BebopJoyNode() : Node("bebop_joy_node") {
    // Suscripción al joystick
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&BebopJoyNode::joyCallback, this, std::placeholders::_1));

    // Publicadores
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/safe_bebop/cmd_vel", 10);
    takeoff_pub_ = this->create_publisher<std_msgs::msg::Empty>("/bebop/takeoff", 10);
    land_pub_ = this->create_publisher<std_msgs::msg::Empty>("/bebop/land", 10);

    // Timer a 30 Hz (33 ms)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&BebopJoyNode::publishCmdVel, this));
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    last_joy_ = *msg;

    // Botón 0: despegue
    if (msg->buttons.size() > 0 && msg->buttons[0]) {
      std_msgs::msg::Empty e;
      takeoff_pub_->publish(e);
    }

    // Botón 1: aterrizaje
    if (msg->buttons.size() > 1 && msg->buttons[1]) {
      std_msgs::msg::Empty e;
      land_pub_->publish(e);
    }
  }

  void publishCmdVel() {
    geometry_msgs::msg::Twist cmd;

    if (!last_joy_.axes.empty()) {
      cmd.linear.x  = last_joy_.axes[1];  // Adelante / atrás
      cmd.linear.y  = last_joy_.axes[0];  // Izquierda / derecha
      cmd.linear.z  = last_joy_.axes[4];  // Subir / bajar
      cmd.angular.z = last_joy_.axes[3];  // Giro yaw
    }

    cmd_pub_->publish(cmd);
  }

  // Miembros
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy last_joy_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BebopJoyNode>());
  rclcpp::shutdown();
  return 0;
}
