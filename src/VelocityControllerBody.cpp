#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <array>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

class InverseDynamicVelocityController : public rclcpp::Node
{
public:
    InverseDynamicVelocityController()
    : Node("inverse_dynamic_velocity_controller")
    {
        using std::placeholders::_1;

        sub_ref_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/nero/vel_ref", 10,
            std::bind(&InverseDynamicVelocityController::ref_callback, this, _1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bebop/odom", 10,
            std::bind(&InverseDynamicVelocityController::odom_callback, this, _1));

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel1", 10);

        timer_ = this->create_wall_timer(20ms, std::bind(&InverseDynamicVelocityController::control_loop, this));

        dX_.fill(0.0);
        last_dX_.fill(0.0);
        acc_.fill(0.0);
        yaw_ = 0.0;
        last_ref_time_ = this->now();
        last_odom_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "InverseDynamicVelocityController completo iniciado (50 Hz)");
    }

private:
    // === Callbacks ===
    void ref_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        ref_ = *msg;
        last_ref_time_ = this->now();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto now = this->now();
        double dt = (now - last_odom_time_).seconds();
        if (dt <= 1e-4) dt = 1e-4;

        // Velocidades actuales
        dX_[0] = msg->twist.twist.linear.x;
        dX_[1] = msg->twist.twist.linear.y;
        dX_[2] = msg->twist.twist.linear.z;
        dX_[3] = msg->twist.twist.angular.z;

        // Aceleración estimada
        const double alpha = 0.8;
        for (int i = 0; i < 3; i++) {
            double a_raw = (dX_[i] - last_dX_[i]) / dt;
            acc_[i] = alpha * acc_[i] + (1.0 - alpha) * a_raw;
            last_dX_[i] = dX_[i];
        }

        // Calcular yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        yaw_ = std::atan2(siny_cosp, cosy_cosp);

        last_odom_time_ = now;
    }

    // === Control principal ===
    void control_loop()
    {
        geometry_msgs::msg::Twist cmd_msg;
        auto now_time = this->now();

        // Hover si no hay referencia reciente
        if ((now_time - last_ref_time_).seconds() > 0.5)
        {
            cmd_msg.linear.x = cmd_msg.linear.y = cmd_msg.linear.z = cmd_msg.angular.z = 0.0;
            pub_cmd_->publish(cmd_msg);
            return;
        }

        // === Referencias ===
        double vx_ref = ref_.linear.x;
        double vy_ref = ref_.linear.y;
        double vz_ref = ref_.linear.z;
        double wz_ref = ref_.angular.z;

        // === Velocidades actuales ===
        double vx = dX_[0];
        double vy = dX_[1];
        double vz = dX_[2];
        double wz = dX_[3];

        // === Transformar error al marco cuerpo ===
        double ex = vx_ref - vx;
        double ey = vy_ref - vy;
        double ez = vz_ref - vz;
        double ew = wz_ref - wz;

        double ex_body =  std::cos(yaw_) * ex + std::sin(yaw_) * ey;
        double ey_body = -std::sin(yaw_) * ex + std::cos(yaw_) * ey;

        // === Ganancias proporcionales ===
        double k_xy = 1.0 / 9.81; // convierte error en pitch/roll
        double k_z  = 1.0 / 6.0;  // convierte error a comando normalizado vertical
        double k_yaw = 1.0 / (200.0 * M_PI / 180.0); // convierte error en comando normalizado

        // === Ángulos deseados ===
        double pitch = k_xy * ex_body;
        double roll  = -k_xy * ey_body;

        // === Saturación angular (±35°) ===
        const double pitch_max = M_PI / 180.0 * 35.0;
        const double roll_max  = M_PI / 180.0 * 35.0;

        // === Normalización ===
        cmd_msg.linear.x = std::clamp(pitch / pitch_max, -1.0, 1.0);
        cmd_msg.linear.y = std::clamp(roll  / roll_max,  -1.0, 1.0);
        cmd_msg.linear.z = std::clamp(k_z * ez, -1.0, 1.0);
        cmd_msg.angular.z = std::clamp(k_yaw * ew, -1.0, 1.0);

        pub_cmd_->publish(cmd_msg);
    }

    // === Variables ===
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_ref_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist ref_;
    std::array<double, 4> dX_;       // [vx, vy, vz, wz]
    std::array<double, 3> acc_;      // [ax, ay, az]
    std::array<double, 4> last_dX_;  // para derivar
    rclcpp::Time last_ref_time_;
    rclcpp::Time last_odom_time_;
    double yaw_;
};

// === MAIN ===
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseDynamicVelocityController>());
    rclcpp::shutdown();
    return 0;
}
