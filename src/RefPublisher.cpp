// RefPublisher: publishes low-speed reference velocities to /nero/vel_ref at 10 Hz for testing the velocity controller. Runs for 10 seconds.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class RefPublisher : public rclcpp::Node {
public:
    RefPublisher() : Node("ref_publisher"), t_(0.0), start_time_(this->now()) {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/nero/vel_ref", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&RefPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Reference velocity publisher started (10 Hz, 10 s runtime).");
    }

private:
    void timer_callback() {
        auto now = this->now();
        double elapsed = (now - start_time_).seconds();
        if (elapsed >= 10.0) {
            RCLCPP_INFO(this->get_logger(), "Reference publishing finished (10 seconds elapsed).");
            rclcpp::shutdown();
            return;
        }

        geometry_msgs::msg::Twist msg;
        msg.linear.x  =  0.15 * std::sin(0.2 * t_);
        msg.linear.y  =  0.15 * std::cos(0.2 * t_);
        msg.linear.z  =  0.05 * std::sin(0.1 * t_);
        msg.angular.z =  0.2  * std::sin(0.25 * t_);
        pub_->publish(msg);
        t_ += 0.1;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double t_;
    rclcpp::Time start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RefPublisher>());
    rclcpp::shutdown();
    return 0;
}
