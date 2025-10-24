#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <array>
#include <cmath>
#include <chrono>
#include <algorithm> // std::clamp

using namespace std::chrono_literals;

class InverseDynamicVelocityController : public rclcpp::Node
{
public:
    InverseDynamicVelocityController()
    : Node("inverse_dynamic_velocity_controller"),
      ref_received_(false)
    {
        // Modelo simplificado (igual al MATLAB)
        model_simp_ = {0.8417, 0.18227, 0.8354, 0.17095, 3.966, 4.001, 9.8524, 4.7295};

        // Ganancias dinámicas
        Ku_ = {model_simp_[0], model_simp_[2], model_simp_[4], model_simp_[6]};
        Kv_ = {model_simp_[1], model_simp_[3], model_simp_[5], model_simp_[7]};

        // Ganancias del controlador
        Ksd_ = {2.0, 2.0, 1.8, 5.0};

        // Suscripciones y publicaciones
        sub_ref_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/nero/vel_ref", 10,
            std::bind(&InverseDynamicVelocityController::ref_callback, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bebop/odom", 10,
            std::bind(&InverseDynamicVelocityController::odom_callback, this, std::placeholders::_1));

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);

        timer_ = this->create_wall_timer(20ms, std::bind(&InverseDynamicVelocityController::control_loop, this));

        last_ref_time_ = this->now();
        last_time_ = this->now();

        // Inicialización de vectores
        Ucw_.fill(0.0);
        Ucw_ant_.fill(0.0);
        dX_.fill(0.0);
        dUcw_.fill(0.0);
        Ud_.fill(0.0);
        yaw_ = 0.0;
    }

private:
    // --- Callback de referencia ---
    void ref_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        Ucw_[0] = msg->linear.x;
        Ucw_[1] = msg->linear.y;
        Ucw_[2] = msg->linear.z;
        Ucw_[3] = msg->angular.z;
        ref_received_ = true;
        last_ref_time_ = this->now();
    }

    // --- Callback de odometría ---
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        dX_[0] = msg->twist.twist.linear.x;
        dX_[1] = msg->twist.twist.linear.y;
        dX_[2] = msg->twist.twist.linear.z;
        dX_[3] = msg->twist.twist.angular.z;

        // Extraer yaw del quaternion
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    // --- Conversión a comandos del Bebop ---
    geometry_msgs::msg::Twist convert_to_bebop_cmd(const std::array<double,4>& Ud)
    {
        geometry_msgs::msg::Twist cmd;

        // Parámetros nominales del Bebop 2
        const double max_tilt_angle_deg = 20.0;    // inclinación máxima (grados)
        const double max_vert_speed = 1.0;         // velocidad vertical máxima (m/s)
        const double max_rot_speed_deg = 100.0;    // velocidad rotacional máxima (grados/s)
        const double max_vxy = 1.0;                // velocidad horizontal máxima esperada (m/s)

        // Conversiones a radianes
        const double max_tilt_rad = max_tilt_angle_deg * M_PI / 180.0;
        const double max_rot_speed_rad = max_rot_speed_deg * M_PI / 180.0;

        // --- Convertir velocidad deseada a ángulo equivalente ---
        double pitch_rad = (Ud[0] / max_vxy) * max_tilt_rad;  // adelante/atrás
        double roll_rad  = (Ud[1] / max_vxy) * max_tilt_rad;  // izquierda/derecha

        // --- Normalizar a [-1, 1] según inclinación máxima ---
        cmd.linear.x  = std::clamp(pitch_rad / max_tilt_rad, -1.0, 1.0);
        cmd.linear.y  = std::clamp(roll_rad  / max_tilt_rad, -1.0, 1.0);

        // --- Escalar velocidad vertical y rotacional ---
        cmd.linear.z  = std::clamp(Ud[2] / max_vert_speed, -1.0, 1.0);
        cmd.angular.z = std::clamp(Ud[3] / max_rot_speed_rad, -1.0, 1.0);

        return cmd;
    }

    // --- Loop de control principal ---
    void control_loop()
    {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0) dt = 0.02;

        // Si no hay referencia reciente, mantener hover
        if ((now - last_ref_time_).seconds() > 0.2)
            ref_received_ = false;

        if (!ref_received_)
        {
            geometry_msgs::msg::Twist hover;
            hover.linear.x = hover.linear.y = hover.linear.z = hover.angular.z = 0.0;
            pub_cmd_->publish(hover);
            last_time_ = now;
            return;
        }

        // Derivada de Ucw (dUcw)
        for (int i = 0; i < 4; ++i)
            dUcw_[i] = (Ucw_[i] - Ucw_ant_[i]) / dt;
        Ucw_ant_ = Ucw_;

        // Matriz de rotación (solo yaw)
        double cy = std::cos(yaw_);
        double sy = std::sin(yaw_);
        double F[4][4] = {
            { cy, -sy, 0, 0 },
            { sy,  cy, 0, 0 },
            {  0,   0, 1, 0 },
            {  0,   0, 0, 1 }
        };

        // Control dinámico inverso
        for (int i = 0; i < 4; ++i)
        {
            double Ku_inv = 1.0 / Ku_[i];
            Ud_[i] = Ku_inv * ( dUcw_[i] + Ksd_[i]*(Ucw_[i] - dX_[i]) + Kv_[i]*dX_[i] );
        }

        // Transformación de marco (inercial → cuerpo)
        std::array<double,4> Ud_body;
        Ud_body[0] = F[0][0]*Ud_[0] + F[0][1]*Ud_[1];
        Ud_body[1] = F[1][0]*Ud_[0] + F[1][1]*Ud_[1];
        Ud_body[2] = Ud_[2];
        Ud_body[3] = Ud_[3];

        // Convertir a comando del Bebop
        auto cmd = convert_to_bebop_cmd(Ud_body);

        // Publicar comando
        pub_cmd_->publish(cmd);
        last_time_ = now;
    }

    // --- ROS interfaces ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_ref_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Variables internas ---
    std::array<double,8> model_simp_;
    std::array<double,4> Ku_, Kv_, Ksd_;
    std::array<double,4> Ucw_, Ucw_ant_, dUcw_, dX_, Ud_;

    double yaw_;
    bool ref_received_;
    rclcpp::Time last_ref_time_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseDynamicVelocityController>());
    rclcpp::shutdown();
    return 0;
}
