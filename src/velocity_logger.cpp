#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <cmath>

class BebopFullLoggerNode : public rclcpp::Node
{
public:
  BebopFullLoggerNode()
  : Node("bebop_full_logger_node")
  {
    // Suscripciones
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&BebopFullLoggerNode::joyCallback, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/bebop/cmd_vel", 10,
      std::bind(&BebopFullLoggerNode::cmdVelCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/bebop/odom", 10,
      std::bind(&BebopFullLoggerNode::odomCallback, this, std::placeholders::_1));

    // Guardará datos una sola vez a los 40 s
    timer_log_once_ = this->create_wall_timer(
      std::chrono::seconds(40),
      std::bind(&BebopFullLoggerNode::saveData, this));

    // Archivo CSV
    log_file_.open("bebop_log.csv", std::ios::out);
    if (log_file_.is_open()) {
      log_file_ << "time_s,"
                << "joy_ax0,joy_ax1,joy_ax2,joy_ax3,joy_ax4,joy_ax5,"
                << "cmd_lin_x,cmd_lin_y,cmd_lin_z,cmd_ang_z,"
                << "odom_x,odom_y,odom_z,odom_yaw,"
                << "odom_lin_x,odom_lin_y,odom_lin_z,"
                << "odom_ang_x,odom_ang_y,odom_ang_z\n";
    }

    start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
                "Logger iniciado. Guardará joystick, cmd_vel y odometría a los 40 s.");
  }

  ~BebopFullLoggerNode()
  {
    if (log_file_.is_open())
      log_file_.close();
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_ = *msg;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
  }

  void saveData()
  {
    double t = (this->now() - start_time_).seconds();

    double qx = last_odom_.pose.pose.orientation.x;
    double qy = last_odom_.pose.pose.orientation.y;
    double qz = last_odom_.pose.pose.orientation.z;
    double qw = last_odom_.pose.pose.orientation.w;
    double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                            1.0 - 2.0 * (qy * qy + qz * qz));

    if (log_file_.is_open()) {
      log_file_ << t << ",";

      // Guardar primeros 6 ejes del joystick
      for (size_t i = 0; i < 6; ++i) {
        if (i < last_joy_.axes.size()) log_file_ << last_joy_.axes[i];
        else log_file_ << 0.0;
        log_file_ << ",";
      }

      // Comandos enviados
      log_file_ << last_cmd_.linear.x << ","
                << last_cmd_.linear.y << ","
                << last_cmd_.linear.z << ","
                << last_cmd_.angular.z << ",";

      // Odometría
      log_file_ << last_odom_.pose.pose.position.x << ","
                << last_odom_.pose.pose.position.y << ","
                << last_odom_.pose.pose.position.z << ","
                << yaw << ","
                << last_odom_.twist.twist.linear.x << ","
                << last_odom_.twist.twist.linear.y << ","
                << last_odom_.twist.twist.linear.z << ","
                << last_odom_.twist.twist.angular.x << ","
                << last_odom_.twist.twist.angular.y << ","
                << last_odom_.twist.twist.angular.z << "\n";
      log_file_.flush();
    }

    RCLCPP_INFO(this->get_logger(),
      "Datos guardados a los %.1f s: joystick, cmd_vel y odometría.", t);

    timer_log_once_->cancel();
  }

  // --- Miembros ---
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_log_once_;
  sensor_msgs::msg::Joy last_joy_;
  geometry_msgs::msg::Twist last_cmd_;
  nav_msgs::msg::Odometry last_odom_;
  std::ofstream log_file_;
  rclcpp::Time start_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BebopFullLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
