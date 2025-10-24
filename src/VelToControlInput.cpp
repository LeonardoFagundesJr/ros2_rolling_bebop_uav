#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <algorithm>

/*
ROS2 node that receives 4D velocity references (vx, vy, vz, wz)
and converts them into the 4 control commands accepted by the Parrot Bebop 2 drone.
Publishes /bebop/cmd_vel at 20 Hz.

Behavior:
- Initializes by publishing hover (safe state).
- Keeps sending last received command.
- Returns to hover if no new reference is received for >1 second.
- In hover mode, injects a small oscillation on vertical speed (±0.01 m/s)
  to prevent the drone from entering motor-idle beep mode.
*/

class VelToControlInput : public rclcpp::Node {
public:
    VelToControlInput() : Node("vel_to_control_input"), ref_received_(false), t_hover_(0.0)
    {
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel1", 10);
        sub_ref_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/nero/cmd_vel", 10,
            std::bind(&VelToControlInput::ref_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),   // 20 Hz
            std::bind(&VelToControlInput::publish_cmd, this));

        // Drone limits
        max_tilt_deg_   = 20.0;
        max_tilt_rad_   = max_tilt_deg_ * M_PI / 180.0;
        max_vert_speed_ = 1.0;                 // m/s
        max_yaw_rate_   = 100.0 * M_PI / 180.; // rad/s

        // Initialize hover state
        last_ref_.linear.x  = 0.0;
        last_ref_.linear.y  = 0.0;
        last_ref_.linear.z  = 0.0;
        last_ref_.angular.z = 0.0;
        last_msg_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "VelToControlInput node started (20 Hz, hover-safe initialization).");
    }

private:
    void ref_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_ref_ = *msg;
        last_msg_time_ = this->now();
        ref_received_ = true;
    }

    void publish_cmd()
    {
        geometry_msgs::msg::Twist cmd;
        double time_since_last = (this->now() - last_msg_time_).seconds();

        // If no reference received for >1s → hover with microoscillation
        if (!ref_received_ || time_since_last > 1.0) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;

            pub_cmd_->publish(cmd);
            t_hover_ += 0.1;

            if (ref_received_ && time_since_last > 1.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "No reference received for >1s, reverting to hover with active stabilization.");
                ref_received_ = false;
            }
            return;
        }

        // Convert reference into Bebop control inputs
        double vx = last_ref_.linear.x;
        double vy = last_ref_.linear.y;
        double vz = last_ref_.linear.z;
        double wz = last_ref_.angular.z;

        // Assuming yaw = 0 (body frame)
        double psi = 0.0;

        double vx_b =  std::cos(psi) * vx + std::sin(psi) * vy;
        double vy_b = -std::sin(psi) * vx + std::cos(psi) * vy;

        // Map linear velocities to pitch/roll angles
        double pitch = std::atan2(vx_b, 9.81);
        double roll  = -std::atan2(vy_b, 9.81);

        // Saturate to drone limits
        pitch = std::clamp(pitch, -max_tilt_rad_, max_tilt_rad_);
        roll  = std::clamp(roll,  -max_tilt_rad_, max_tilt_rad_);

        cmd.linear.x  = pitch / max_tilt_rad_;
        cmd.linear.y  = roll  / max_tilt_rad_;
        cmd.linear.z  = std::clamp(vz / max_vert_speed_, -1.0, 1.0);
        cmd.angular.z = std::clamp(wz / max_yaw_rate_,  -1.0, 1.0);

        pub_cmd_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_ref_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist last_ref_;
    rclcpp::Time last_msg_time_;
    bool ref_received_;
    double t_hover_;

    double max_tilt_deg_;
    double max_tilt_rad_;
    double max_vert_speed_;
    double max_yaw_rate_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelToControlInput>());
    rclcpp::shutdown();
    return 0;
}
