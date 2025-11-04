// TagTFDetector (with RViz sphere marker)
// Author: Brayan Saldarriaga-Mesa

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>

class TagTFDetector : public rclcpp::Node {
public:
    TagTFDetector() : Node("tag_tf_detector"),
                      tf_buffer_(this->get_clock()),
                      tf_listener_(tf_buffer_)
    {
        // --- AprilTag setup ---
        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);

        // --- ROS setup ---
        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/bebop/camera/camera_info", 10,
            std::bind(&TagTFDetector::camera_info_callback, this, std::placeholders::_1));

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/bebop/camera/image_raw", 10,
            std::bind(&TagTFDetector::image_callback, this, std::placeholders::_1));

        reference_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tag_reference_odom", 10);
        detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/bebop/detected", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("tag_marker", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        tag_size_ = 0.145;
        RCLCPP_INFO(this->get_logger(), "TagTFDetector initialized (with RViz marker).");
    }

    ~TagTFDetector() {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (has_camera_info_) return;
        cameraMatrix_ = (cv::Mat1d(3,3) <<
            msg->k[0], msg->k[1], msg->k[2],
            msg->k[3], msg->k[4], msg->k[5],
            msg->k[6], msg->k[7], msg->k[8]);
        distCoeffs_ = cv::Mat(msg->d).clone();
        has_camera_info_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera intrinsics stored.");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!has_camera_info_) return;

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) { return; }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = { gray.cols, gray.rows, gray.cols, gray.data };
        zarray_t* detections = apriltag_detector_detect(td_, &im);

        bool tag_detected = false;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);
            if (det->id != 0) continue;
            tag_detected = true;

            double s = tag_size_ / 2.0;
            std::vector<cv::Point3f> objectPoints = {
                {-s,-s,0}, {s,-s,0}, {s,s,0}, {-s,s,0}
            };
            std::vector<cv::Point2f> imagePoints = {
                {(float)det->p[0][0], (float)det->p[0][1]},
                {(float)det->p[1][0], (float)det->p[1][1]},
                {(float)det->p[2][0], (float)det->p[2][1]},
                {(float)det->p[3][0], (float)det->p[3][1]}
            };

            cv::Mat rvec, tvec;
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix_, distCoeffs_, rvec, tvec);
            cv::Mat R_tag2cam;
            cv::Rodrigues(rvec, R_tag2cam);

            // --- Rotación corregida ---
            cv::Mat R_y180 = (cv::Mat1d(3,3) << -1,0,0, 0,1,0, 0,0,-1);
            cv::Mat R_z180 = (cv::Mat1d(3,3) << -1,0,0, 0,-1,0, 0,0,1);
            cv::Mat R_fix = (cv::Mat1d(3,3) << 1,0,0, 0,-1,0, 0,0,-1);
            cv::Mat R_corrected = R_fix * R_z180 * R_y180 * R_tag2cam;

            tf2::Matrix3x3 tf2_rot(
                R_corrected.at<double>(0,0), R_corrected.at<double>(0,1), R_corrected.at<double>(0,2),
                R_corrected.at<double>(1,0), R_corrected.at<double>(1,1), R_corrected.at<double>(1,2),
                R_corrected.at<double>(2,0), R_corrected.at<double>(2,1), R_corrected.at<double>(2,2)
            );
            tf2::Quaternion q_tag;
            tf2_rot.getRotation(q_tag);
            q_tag.normalize();

            // --- Publicar TF del tag ---
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = this->now();
            tf_msg.header.frame_id = "camera_gimbal";
            tf_msg.child_frame_id = "tag_0";
            tf_msg.transform.translation.x = tvec.at<double>(0);
            tf_msg.transform.translation.y = tvec.at<double>(1);
            tf_msg.transform.translation.z = tvec.at<double>(2);
            tf_msg.transform.rotation.x = q_tag.x();
            tf_msg.transform.rotation.y = q_tag.y();
            tf_msg.transform.rotation.z = q_tag.z();
            tf_msg.transform.rotation.w = q_tag.w();
            tf_broadcaster_->sendTransform(tf_msg);

            // --- Dibujar esfera en RViz ---
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "camera_gimbal";  // o "odom" si prefieres marco global
            marker.header.stamp = this->now();
            marker.ns = "tag_marker";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Centro de la esfera en la posición del tag
            marker.pose.position.x = tvec.at<double>(0);
            marker.pose.position.y = tvec.at<double>(1);
            marker.pose.position.z = tvec.at<double>(2);
            marker.pose.orientation.w = 1.0;

            // Escala de 1 m (diámetro)
            marker.scale.x = 1.5;
            marker.scale.y = 1.5;
            marker.scale.z = 1.5;

            // Color semitransparente
            marker.color.r = 0.0f;
            marker.color.g = 0.5f;
            marker.color.b = 1.0f;
            marker.color.a = 0.6f;

            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_pub_->publish(marker);

            // --- Calcular referencia ---
            cv::Mat P_ref_tag = (cv::Mat1d(3,1) << 0.0, 0.3, 1.5);
            cv::Mat P_ref_cam = R_corrected * P_ref_tag + tvec;

            geometry_msgs::msg::PoseStamped pose_cam;
            pose_cam.header.frame_id = "camera_gimbal";
            pose_cam.pose.position.x = P_ref_cam.at<double>(0);
            pose_cam.pose.position.y = P_ref_cam.at<double>(1);
            pose_cam.pose.position.z = P_ref_cam.at<double>(2);
            pose_cam.pose.orientation.w = 1.0;

            geometry_msgs::msg::PoseStamped pose_odom;
            try {
                pose_odom = tf_buffer_.transform(pose_cam, "odom", tf2::durationFromSec(0.02));
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Transform error: %s", ex.what());
                continue;
            }

            // --- Publicar referencia inmediata ---
            pose_odom.header.stamp = this->now();
            reference_pub_->publish(pose_odom);
        }

        std_msgs::msg::Bool det;
        det.data = tag_detected;
        detected_pub_->publish(det);
        apriltag_detections_destroy(detections);
    }

    // --- ROS y TF ---
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    apriltag_family_t *tf_;
    apriltag_detector_t *td_;
    cv::Mat cameraMatrix_, distCoeffs_;
    bool has_camera_info_ = false;
    double tag_size_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagTFDetector>());
    rclcpp::shutdown();
    return 0;
}
