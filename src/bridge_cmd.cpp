// safe_bebop_republisher.cpp
// Escucha /safe_bebop/cmd_vel y repubblica en /bebop/cmd_vel
// Autor: Brayan Saldarriaga-Mesa, 2025

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class SafeBebopRepublisher : public rclcpp::Node {
public:
  SafeBebopRepublisher() : Node("safe_bebop_republisher") {
    // Suscriptor al tópico seguro
    safe_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/safe_bebop/cmd_vel",
      10,
      std::bind(&SafeBebopRepublisher::safeCmdCallback, this, std::placeholders::_1)
    );

    // Publicador al tópico real del dron
    bebop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo safe_bebop_republisher iniciado. Escuchando /safe_bebop/cmd_vel y reenviando a /bebop/cmd_vel");
  }

private:
  void safeCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Reenviar directamente el mensaje recibido
    bebop_pub_->publish(*msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr safe_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr bebop_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafeBebopRepublisher>());
  rclcpp::shutdown();
  return 0;
}
