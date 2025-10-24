#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class CameraRotZXTF : public rclcpp::Node {
public:
    CameraRotZXTF() : Node("camera_rot_zx_tf") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publicar a 20 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&CameraRotZXTF::publish_tf, this));

        RCLCPP_INFO(this->get_logger(),
                    "Camera TF node started (rotación -90° Z, luego +90° X).");
    }

private:
    void publish_tf() {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "camera_link";

        // --- Traslación física ---
        tf_msg.transform.translation.x = 0.09;  // 9 cm hacia adelante
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.02;  // 2 cm arriba

        // --- Rotaciones ---
        tf2::Quaternion q_z, q_x, q_total;

        // Primera rotación: -90° alrededor de Z
        q_z.setRPY(0.0, 0.0, -M_PI_2);

        // Segunda rotación: +90° alrededor de X
        q_x.setRPY(0.0, M_PI_2, 0.0);

        // Composición: aplicar primero Z, luego X
        q_total = q_x * q_z;
        q_total.normalize();

        tf_msg.transform.rotation.x = q_total.x();
        tf_msg.transform.rotation.y = q_total.y();
        tf_msg.transform.rotation.z = q_total.z();
        tf_msg.transform.rotation.w = q_total.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraRotZXTF>());
    rclcpp::shutdown();
    return 0;
}
