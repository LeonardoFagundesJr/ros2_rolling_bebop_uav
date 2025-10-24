#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>

class BebopJoyNode : public rclcpp::Node
{
public:
  BebopJoyNode()
  : Node("bebop_joy_node")
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&BebopJoyNode::joyCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);
    takeoff_pub_ = this->create_publisher<std_msgs::msg::Empty>("/bebop/takeoff", 10);
    land_pub_ = this->create_publisher<std_msgs::msg::Empty>("/bebop/land", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),   // 100 Hz
      std::bind(&BebopJoyNode::publishCmdVel, this));

    RCLCPP_INFO(this->get_logger(), "Bebop Joy Node iniciado a 50 Hz.");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Guardamos última lectura de ejes
    last_joy_ = *msg;

    // Botón A → Takeoff, Botón B → Land (índices dependen del joystick)
    // Suponiendo: A = 0, B = 1
    if (msg->buttons.size() > 0 && msg->buttons[0])
    {
      std_msgs::msg::Empty e;
      takeoff_pub_->publish(e);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Takeoff enviado");
    }
    if (msg->buttons.size() > 1 && msg->buttons[1])
    {
      std_msgs::msg::Empty e;
      land_pub_->publish(e);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Land enviado");
    }
  }

  void publishCmdVel()
  {
    geometry_msgs::msg::Twist cmd;

    if (!last_joy_.axes.empty())
    {
      // Escala directa [-1, 1]
      // Orden: linear.x, linear.y, linear.z, angular.z
        cmd.linear.x  = last_joy_.axes[1];  // Adelante/atrás
    cmd.linear.y  = last_joy_.axes[0];  // Izquierda/derecha
    cmd.linear.z  = last_joy_.axes[4];  // Altura
    cmd.angular.z = last_joy_.axes[3];  // Rotación

    }

    cmd_pub_->publish(cmd);
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy last_joy_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BebopJoyNode>());
  rclcpp::shutdown();
  return 0;
}
