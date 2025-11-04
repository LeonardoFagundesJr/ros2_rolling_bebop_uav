// TopicLogger: Nodo ROS2 para registrar odometría, comandos y referencias del Bebop
// Las callbacks solo actualizan variables auxiliares.
// Un temporizador (30 Hz) registra todo de forma sincronizada.
// Autor: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar)

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <mutex>

class TopicLogger : public rclcpp::Node
{
public:
    TopicLogger()
        : Node("topic_logger_node"),
          first_sample_(true),
          ref_received_(false),
          counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Nodo de registro iniciado (30 Hz con variables auxiliares)");

        // ------------------- Suscripciones -------------------
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bebop/odom", 10,
            std::bind(&TopicLogger::odomCallback, this, std::placeholders::_1));

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/safe_bebop/cmd_vel", 20,
            std::bind(&TopicLogger::cmdCallback, this, std::placeholders::_1));

        sub_ref_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/bebop/ref_vec", 10,
            std::bind(&TopicLogger::refCallback, this, std::placeholders::_1));

        // ------------------- Archivo CSV -------------------
        logfile_.open("bebop_log.csv", std::ios::out);
        logfile_ << "time,"
                 << "x,y,z,roll,pitch,yaw,"
                 << "linx,liny,linz,angx,angy,angz,"
                 << "cmd_linx,cmd_liny,cmd_linz,cmd_angz,"
                 << "yaw_pos,yaw_rate,"
                 << "xd,yd,zd,yawd,vxd,vyd,vzd,wyawd\n";

        // ------------------- Temporizador 30 Hz -------------------
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 30.0),
            std::bind(&TopicLogger::logData, this));
    }

    ~TopicLogger()
    {
        logfile_.close();
    }

private:
    std::mutex mtx_;

    // ------------------- Callbacks -------------------
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_odom_ = *msg;

        const auto &pose = msg->pose.pose;
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        double t = this->now().seconds();

        if (!first_sample_)
        {
            double dyaw = yaw - last_yaw_;
            if (dyaw > M_PI)
                dyaw -= 2 * M_PI;
            else if (dyaw < -M_PI)
                dyaw += 2 * M_PI;

            double dt = t - last_time_;
            if (dt > 1e-4)
            {
                last_yaw_rate_ = dyaw / dt;
            }
            // Si dt es muy pequeño, se conserva el valor anterior de yaw_rate
        }
        else
        {
            first_sample_ = false;
            last_yaw_rate_ = 0.0;
        }

        last_yaw_ = yaw;
        last_time_ = t;
    }

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_cmd_ = *msg;
    }

    void refCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 8)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            last_ref_ = msg->data;
            ref_received_ = true;
        }
    }

    // ------------------- Registro periódico -------------------
    void logData()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        double t = this->now().seconds();

        logfile_ << std::fixed << std::setprecision(6) << t << ",";

        // Odometría
        logfile_ << last_odom_.pose.pose.position.x << ","
                 << last_odom_.pose.pose.position.y << ","
                 << last_odom_.pose.pose.position.z << ",";

        tf2::Quaternion q(last_odom_.pose.pose.orientation.x,
                          last_odom_.pose.pose.orientation.y,
                          last_odom_.pose.pose.orientation.z,
                          last_odom_.pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        logfile_ << roll << "," << pitch << "," << yaw << ",";

        logfile_ << last_odom_.twist.twist.linear.x << ","
                 << last_odom_.twist.twist.linear.y << ","
                 << last_odom_.twist.twist.linear.z << ","
                 << last_odom_.twist.twist.angular.x << ","
                 << last_odom_.twist.twist.angular.y << ","
                 << last_odom_.twist.twist.angular.z << ",";

        // Comandos
        logfile_ << last_cmd_.linear.x << "," << last_cmd_.linear.y << "," << last_cmd_.linear.z << ","
                 << last_cmd_.angular.z << ",";

        // Yaw y yaw_rate calculados
        logfile_ << last_yaw_ << "," << last_yaw_rate_ << ",";

        // Referencias
        if (ref_received_)
        {
            for (size_t i = 0; i < last_ref_.size(); ++i)
            {
                logfile_ << last_ref_[i];
                if (i < last_ref_.size() - 1)
                    logfile_ << ",";
            }
        }
        else
        {
            for (int i = 0; i < 8; ++i)
            {
                logfile_ << 0.0;
                if (i < 7)
                    logfile_ << ",";
            }
        }

        logfile_ << "\n";
        if (++counter_ % 30 == 0)
            logfile_.flush();
    }

    // ------------------- Miembros -------------------
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ref_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry last_odom_;
    geometry_msgs::msg::Twist last_cmd_;

    std::ofstream logfile_;
    int counter_;

    // Estado yaw
    double last_yaw_;
    double last_yaw_rate_;
    double last_time_;
    bool first_sample_;

    bool ref_received_;
    std::vector<double> last_ref_;
};

// ------------------- MAIN -------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicLogger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
