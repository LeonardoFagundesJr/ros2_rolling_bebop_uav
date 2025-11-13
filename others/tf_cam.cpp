#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.hpp>

class VirtualGimbalTF : public rclcpp::Node {
public:
    VirtualGimbalTF() : Node("virtual_gimbal_tf") {
        // Suscripción a la odometría del Bebop
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bebop/odom", 10,
            std::bind(&VirtualGimbalTF::odom_callback, this, std::placeholders::_1));

        // Publicador de TF
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "Virtual Gimbal TF node started.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // --- Obtener la orientación del dron (quaternion) ---
        tf2::Quaternion q_body;
        q_body.setX(msg->pose.pose.orientation.x);
        q_body.setY(msg->pose.pose.orientation.y);
        q_body.setZ(msg->pose.pose.orientation.z);
        q_body.setW(msg->pose.pose.orientation.w);

        // --- Convertir a ángulos de Euler ---
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_body).getRPY(roll, pitch, yaw);

        // --- Modelo de gimbal virtual del Bebop ---
        // Compensa roll y pitch (nivelación del horizonte), mantiene yaw (rumbo)
        roll  = -roll;
        pitch = -pitch;

        tf2::Quaternion q_virtual;
        q_virtual.setRPY(roll, pitch, yaw);
        q_virtual.normalize();

        // --- Crear el mensaje TF ---
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = "base_link";      // marco del dron
        tf_msg.child_frame_id = "camera_virtual";  // cámara estabilizada

        // --- Posición física de la cámara respecto al cuerpo del dron ---
        tf_msg.transform.translation.x = 0.09;   // 9 cm hacia adelante
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.02;   // 2 cm hacia arriba

        // --- Orientación compensada del gimbal virtual ---
        tf_msg.transform.rotation.x = q_virtual.x();
        tf_msg.transform.rotation.y = q_virtual.y();
        tf_msg.transform.rotation.z = q_virtual.z();
        tf_msg.transform.rotation.w = q_virtual.w();

        // --- Publicar TF ---
        tf_broadcaster_->sendTransform(tf_msg);

        // --- Log informativo (limitado a 2 Hz) ---
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Virtual Gimbal (deg): R=%.2f | P=%.2f | Y=%.2f",
            roll * 180.0 / M_PI,
            pitch * 180.0 / M_PI,
            yaw * 180.0 / M_PI);
    }

    // --- Componentes ROS2 ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualGimbalTF>());
    rclcpp::shutdown();
    return 0;
}
