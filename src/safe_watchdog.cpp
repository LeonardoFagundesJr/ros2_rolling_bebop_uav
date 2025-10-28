// SafetyWatchdogROS2
// Autor: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar), 2025
// Nodo de seguridad con lógica de vuelo: publica cero si no se reciben cmd_vel a 10 Hz,
// pero sólo cuando el dron está en vuelo (tras /bebop/takeoff) y sin /bebop/land activo.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SafetyWatchdog : public rclcpp::Node {
public:
  SafetyWatchdog()
  : Node("safety_watchdog"),
    last_cmd_time_(this->now()),
    timeout_(0.1),
    in_flight_(false),
    has_received_cmd_(false)
  {
    // --- Publishers ---
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);

    // --- Subscribers ---
    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/bebop/cmd_vel", 10,
      std::bind(&SafetyWatchdog::cmdCallback, this, std::placeholders::_1));

    sub_takeoff_ = this->create_subscription<std_msgs::msg::Empty>(
      "/bebop/takeoff", 10,
      std::bind(&SafetyWatchdog::takeoffCallback, this, std::placeholders::_1));

    sub_land_ = this->create_subscription<std_msgs::msg::Empty>(
      "/bebop/land", 10,
      std::bind(&SafetyWatchdog::landCallback, this, std::placeholders::_1));

    // --- Timer ---
    timer_ = this->create_wall_timer(20ms, std::bind(&SafetyWatchdog::checkTimeout, this));

    RCLCPP_INFO(this->get_logger(), "Safety Watchdog initialized (timeout = %.2f s)", timeout_);
  }

private:
  // --- Callbacks ---
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr /*msg*/) {
    last_cmd_time_ = this->now();
    has_received_cmd_ = true;
  }

  void takeoffCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
    in_flight_ = true;
    has_received_cmd_ = false;  // reset when takeoff starts
    RCLCPP_INFO(this->get_logger(), "Takeoff detected — watchdog active");
  }

  void landCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
    in_flight_ = false;
    publishZero();  // ensure stop at landing
    RCLCPP_INFO(this->get_logger(), "Landing detected — watchdog disabled");
  }

  void checkTimeout() {
    if (!in_flight_ || !has_received_cmd_)
      return;  // do nothing if not flying or no cmd yet

    double elapsed = (this->now() - last_cmd_time_).seconds();
    if (elapsed > timeout_) {
      publishZero();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Timeout: no cmd_vel received for %.3f s — sending STOP", elapsed);
    }
  }

  void publishZero() {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = stop.linear.y = stop.linear.z = stop.angular.z = 0.0;
    cmd_pub_->publish(stop);
  }

  // --- ROS interfaces ---
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_takeoff_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_land_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Estado interno ---
  rclcpp::Time last_cmd_time_;
  double timeout_;
  bool in_flight_;
  bool has_received_cmd_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyWatchdog>());
  rclcpp::shutdown();
  return 0;
}
