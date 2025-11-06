#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SafetyWatchdog : public rclcpp::Node {
public:
  SafetyWatchdog()
  : Node("safety_watchdog"),
    last_cmd_time_(this->now()),
    timeout_(0.2),
    stopped_(false),
    flying_(false),
    recovered_(false)
  {
    // --- Suscripciones ---
    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/safe_bebop/cmd_vel", 10,
      std::bind(&SafetyWatchdog::cmdCallback, this, std::placeholders::_1));

    // Nuevo: escucha estado booleano de vuelo
    sub_is_flying_ = this->create_subscription<std_msgs::msg::Bool>(
      "/bebop/is_flying", 10,
      std::bind(&SafetyWatchdog::isFlyingCallback, this, std::placeholders::_1));

    // Publicador de comandos finales
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);
    
    // Timer de verificación
    timer_ = this->create_wall_timer(40ms, std::bind(&SafetyWatchdog::checkTimeout, this));

    RCLCPP_INFO(this->get_logger(),
                "SafetyWatchdog iniciado (timeout = %.2f s). Esperando flag /bebop/is_flying.",
                timeout_);
  }

private:
  // === CALLBACKS ===
  void isFlyingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    bool prev = flying_;
    flying_ = msg->data;

    if (flying_ && !prev) {
      RCLCPP_INFO(this->get_logger(), "Flag is_flying TRUE — watchdog activado.");
      last_cmd_time_ = this->now();
      stopped_ = false;
      recovered_ = false;
    }
    else if (!flying_ && prev) {
      RCLCPP_INFO(this->get_logger(), "Flag is_flying FALSE — watchdog desactivado.");
      stopped_ = false;
      recovered_ = false;
      publishZero();
    }
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_is_flying_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Estado interno ---
  rclcpp::Time last_cmd_time_;
  double timeout_;
  bool stopped_;
  bool flying_;
  bool recovered_;
};

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyWatchdog>());
  rclcpp::shutdown();
  return 0;
}
