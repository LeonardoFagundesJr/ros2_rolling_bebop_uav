#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;

class BebopPositionController : public rclcpp::Node
{
public:
    BebopPositionController() : Node("bebop_position_controller")
    {
        // PID gains
        kp_ << 1.0, 1.0, 1.2;
        ki_ << 0.1, 0.1, 0.2;
        kd_ << 0.5, 0.5, 0.6;

        int_error_.setZero();
        prev_error_.setZero();

        // Publishers & subscribers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/bebop/force_marker", 10);

        ref_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose/ref", 10,
            std::bind(&BebopPositionController::refCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bebop/odom", 10,
            std::bind(&BebopPositionController::odomCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(20ms, std::bind(&BebopPositionController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Bebop PID Position Controller iniciado (50 Hz)");
    }

private:
    // ---- Callbacks ----
    void refCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Actualizamos la referencia de forma dinámica
        ref_pos_ << msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z;
        ref_received_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        curr_pos_ << msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z;

        // Extraer yaw desde cuaternión
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    // ---- Control Loop ----
    void controlLoop()
    {
        const double dt = 0.02; // 50 Hz

        if (!ref_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Esperando referencia en /pose/ref...");
            return;
        }

        // Error en marco del mundo
        Eigen::Vector3d e = ref_pos_ - curr_pos_;

        // Rotación mundo→cuerpo
        Eigen::Matrix3d Rz_T;
        Rz_T <<  std::cos(yaw_),  std::sin(yaw_), 0,
                -std::sin(yaw_),  std::cos(yaw_), 0,
                 0,               0,              1;

        Eigen::Vector3d e_b = Rz_T * e;
        Eigen::Vector3d de_b = (e_b - prev_error_) / dt;
        int_error_ += e_b * dt;

        // PID
        Eigen::Vector3d v_b = kp_.cwiseProduct(e_b)
                            + ki_.cwiseProduct(int_error_)
                            + kd_.cwiseProduct(de_b);

        // Saturación ±1 m/s
        v_b = v_b.cwiseMax(Eigen::Vector3d(-1.0, -1.0, -1.0))
                   .cwiseMin(Eigen::Vector3d(1.0, 1.0, 1.0));

        // Publicar comando
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v_b[0];
        cmd.linear.y = v_b[1];
        cmd.linear.z = v_b[2];
        cmd.angular.z = 0.0; // no controlamos yaw
        cmd_pub_->publish(cmd);

        // Publicar marcador visual de la dirección del comando
        publishForceMarker(v_b);

        prev_error_ = e_b;
    }

    // ---- Marcador visual para RViz ----
    void publishForceMarker(const Eigen::Vector3d &v_b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; // vector en marco del cuerpo
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "bebop_control";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Escala más delgada (flecha fina)
        double magnitude = v_b.norm();
        marker.scale.x = std::min(1.0, magnitude); // longitud proporcional
        marker.scale.y = 0.01; // grosor del eje (antes 0.05)
        marker.scale.z = 0.01; // grosor del cono (antes 0.05)

        // Color (verde -> rojo según magnitud)
        marker.color.a = 1.0;
        marker.color.r = std::min(1.0, magnitude);
        marker.color.g = 1.0 - std::min(1.0, magnitude);
        marker.color.b = 0.0;

        // Puntos del marcador
        geometry_msgs::msg::Point start, end;
        start.x = 0.0;
        start.y = 0.0;
        start.z = 0.0;
        end.x = v_b[0];
        end.y = v_b[1];
        end.z = v_b[2];
        marker.points = {start, end};

        // Duración corta para actualizar dinámicamente
        marker.lifetime = rclcpp::Duration::from_seconds(0.05);

        marker_pub_->publish(marker);
    }

    // ---- Variables ----
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    Eigen::Vector3d ref_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d curr_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d int_error_;
    Eigen::Vector3d prev_error_;
    Eigen::Vector3d kp_, ki_, kd_;
    double yaw_ = 0.0;
    bool ref_received_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BebopPositionController>());
    rclcpp::shutdown();
    return 0;
}
