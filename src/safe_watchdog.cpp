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
    timeout_(0.4),
    stopped_(false),
    flying_(false),
    recovered_(false)
  {
    // --- Suscripciones ---
    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/safe_bebop/cmd_vel", 10,
      std::bind(&SafetyWatchdog::cmdCallback, this, std::placeholders::_1));

    sub_takeoff_ = this->create_subscription<std_msgs::msg::Empty>(
      "/bebop/takeoff", 10,
      std::bind(&SafetyWatchdog::takeoffCallback, this, std::placeholders::_1));

    sub_land_ = this->create_subscription<std_msgs::msg::Empty>(
      "/bebop/land", 10,
      std::bind(&SafetyWatchdog::landCallback, this, std::placeholders::_1));

    // --- Publicador ---
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);

    // --- Timer de vigilancia ---
    timer_ = this->create_wall_timer(40ms, std::bind(&SafetyWatchdog::checkTimeout, this));

    RCLCPP_INFO(this->get_logger(),
                "SafetyWatchdog iniciado (timeout = %.2f s). Esperando /bebop/takeoff para activarse.",
                timeout_);
  }

private:
  // === CALLBACKS ===
  void takeoffCallback(const std_msgs::msg::Empty::SharedPtr) {
    flying_ = true;
    stopped_ = false;
    recovered_ = false;
    last_cmd_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Takeoff detectado — watchdog activado.");
  }

  void landCallback(const std_msgs::msg::Empty::SharedPtr) {
    flying_ = false;
    stopped_ = false;
    recovered_ = false;
    publishZero();
    RCLCPP_INFO(this->get_logger(), "Land detectado — watchdog desactivado.");
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!flying_) return;  // ignora si no está volando

    last_cmd_time_ = this->now();

    // Si estaba detenido por timeout y ahora volvió a recibir comandos
    if (stopped_) {
      stopped_ = false;
      recovered_ = true;
      RCLCPP_INFO(this->get_logger(), "Comandos restaurados — watchdog reactivado.");
    } else {
      recovered_ = false;
    }

    cmd_pub_->publish(*msg);
  }

  // === FUNCIÓN DE MONITOREO ===
  void checkTimeout() {
    if (!flying_) return;

    double elapsed = (this->now() - last_cmd_time_).seconds();

    if (elapsed > timeout_) {
      if (!stopped_) {
        publishZero();
        stopped_ = true;
        recovered_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "Timeout: no cmd_vel recibido por %.3f s → enviando STOP", elapsed);
      } else {
        publishZero();  // sigue enviando stop
      }
    }
  }

  // === FUNCIONES AUXILIARES ===
  void publishZero() {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = stop.linear.y = stop.linear.z = stop.angular.z = 0.0;
    cmd_pub_->publish(stop);
  }

  // --- ROS interfaces ---
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_takeoff_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_land_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Estado interno ---
  rclcpp::Time last_cmd_time_;
  double timeout_;
  bool stopped_;
  bool flying_;
  bool recovered_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyWatchdog>());
  rclcpp::shutdown();
  return 0;
}
