// CameraTFWithGimbal: ROS2 node that publishes two TFs — camera_link (fixed mount) and camera_gimbal (stabilized).
// The gimbal compensates roll fully and pitch within physical limits (-80° to +20°).
// Author: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar), in collaboration with UFV.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <algorithm>

class CameraTFWithGimbal : public rclcpp::Node {
public:
    CameraTFWithGimbal() : Node("camera_tf_with_gimbal") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bebop/odom", 10,
            std::bind(&CameraTFWithGimbal::odom_callback, this, std::placeholders::_1));

        min_pitch_ = -80.0 * M_PI / 180.0;
        max_pitch_ =  20.0 * M_PI / 180.0;

        RCLCPP_INFO(this->get_logger(),
                    "Camera TF with gimbal started (pitch limited -80°/+20°, roll free).");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped tf_cam;
        tf_cam.header.stamp = this->get_clock()->now();
        tf_cam.header.frame_id = "base_link";
        tf_cam.child_frame_id = "camera_link";

        tf_cam.transform.translation.x = 0.09;
        tf_cam.transform.translation.y = 0.0;
        tf_cam.transform.translation.z = 0.02;

        tf2::Quaternion q_z, q_x, q_cam;
        q_z.setRPY(0.0, 0.0, -M_PI_2);
        q_x.setRPY(0.0, M_PI_2, 0.0);
        q_cam = q_x * q_z;
        q_cam.normalize();

        tf_cam.transform.rotation.x = q_cam.x();
        tf_cam.transform.rotation.y = q_cam.y();
        tf_cam.transform.rotation.z = q_cam.z();
        tf_cam.transform.rotation.w = q_cam.w();
        tf_broadcaster_->sendTransform(tf_cam);

        tf2::Quaternion q_drone(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_drone).getRPY(roll, pitch, yaw);
        double limited_pitch = std::clamp(pitch, min_pitch_, max_pitch_);

        tf2::Quaternion q_pitch_comp, q_roll_comp, q_total_comp;
        q_pitch_comp.setRPY(limited_pitch, 0.0, 0.0);
        q_roll_comp.setRPY(0.0, 0.0, -roll);
        q_total_comp = q_roll_comp * q_pitch_comp;
        q_total_comp.normalize();

        geometry_msgs::msg::TransformStamped tf_gimbal;
        tf_gimbal.header.stamp = this->get_clock()->now();
        tf_gimbal.header.frame_id = "camera_link";
        tf_gimbal.child_frame_id = "camera_gimbal";

        tf_gimbal.transform.translation.x = 0.0;
        tf_gimbal.transform.translation.y = 0.0;
        tf_gimbal.transform.translation.z = 0.0;

        tf_gimbal.transform.rotation.x = q_total_comp.x();
        tf_gimbal.transform.rotation.y = q_total_comp.y();
        tf_gimbal.transform.rotation.z = q_total_comp.z();
        tf_gimbal.transform.rotation.w = q_total_comp.w();
        tf_broadcaster_->sendTransform(tf_gimbal);
    }

    double min_pitch_;
    double max_pitch_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraTFWithGimbal>());
    rclcpp::shutdown();
    return 0;
}
