// TagTFDetector: ROS2 node that detects AprilTags using OpenCV and AprilTag 3,
// estimates the 3D pose of tag_0 relative to the gimbal-mounted camera (camera_gimbal frame),
// applies 180° around Y (Z forward) and 180° around Z (Y up),
// and publishes TF, odom reference, and visualization marker.
//
// Author: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar), in collaboration with UFV.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class TagTFDetector : public rclcpp::Node {
public:
    TagTFDetector() : Node("tag_tf_detector"),
                      tf_buffer_(this->get_clock()),
                      tf_listener_(tf_buffer_)
    {
        // Inicialización de AprilTag
        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);

        // Suscripciones y publicaciones
        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/bebop/camera/camera_info", 10,
            std::bind(&TagTFDetector::camera_info_callback, this, std::placeholders::_1));

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/bebop/camera/image_raw", 10,
            std::bind(&TagTFDetector::image_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tag_pose", 10);
        reference_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tag_reference_odom", 10);
        ref_vec_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bebop/ref_vec", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("tag_reference_marker", 10);
        detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/bebop/detected", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        tag_size_ = 0.145;  // metros
        alpha_ = 0.85;
        first_detection_ = true;

        RCLCPP_INFO(this->get_logger(), "TagTFDetector initialized (Y=180°, Z=180°, odom reference).");
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
        } catch (cv_bridge::Exception &e) {
            RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

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

            // --- Rotaciones personalizadas ---
            cv::Mat R_y180 = (cv::Mat1d(3,3) <<
                -1, 0,  0,
                 0, 1,  0,
                 0, 0, -1
            );

            cv::Mat R_z180 = (cv::Mat1d(3,3) <<
                -1,  0,  0,
                 0, -1,  0,
                 0,  0,  1
            );

            cv::Mat R_fix = (cv::Mat1d(3,3) <<
                 1,  0,  0,
                 0, -1,  0,
                 0,  0, -1
            );

            // Aplica todas las rotaciones en orden
            cv::Mat R_corrected = R_fix * R_z180 * R_y180 * R_tag2cam;

            // --- Convertir a cuaternión ---
            tf2::Matrix3x3 tf2_rot(
                R_corrected.at<double>(0,0), R_corrected.at<double>(0,1), R_corrected.at<double>(0,2),
                R_corrected.at<double>(1,0), R_corrected.at<double>(1,1), R_corrected.at<double>(1,2),
                R_corrected.at<double>(2,0), R_corrected.at<double>(2,1), R_corrected.at<double>(2,2)
            );
            tf2::Quaternion q_tag;
            tf2_rot.getRotation(q_tag);
            q_tag.normalize();

            cv::Point3d pos_curr(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

            if (first_detection_) {
                prev_pos_ = pos_curr;
                prev_q_ = q_tag;
                first_detection_ = false;
            }
            cv::Point3d pos_filt = alpha_ * prev_pos_ + (1 - alpha_) * pos_curr;
            tf2::Quaternion q_filt = prev_q_.slerp(q_tag, 1.0 - alpha_);
            q_filt.normalize();
            prev_pos_ = pos_filt;
            prev_q_ = q_filt;

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "camera_gimbal";
            pose_msg.pose.position.x = pos_filt.x;
            pose_msg.pose.position.y = pos_filt.y;
            pose_msg.pose.position.z = pos_filt.z;
            pose_msg.pose.orientation.x = q_filt.x();
            pose_msg.pose.orientation.y = q_filt.y();
            pose_msg.pose.orientation.z = q_filt.z();
            pose_msg.pose.orientation.w = q_filt.w();
            pose_pub_->publish(pose_msg);

            // TF del tag
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = this->now();
            tf_msg.header.frame_id = "camera_gimbal";
            tf_msg.child_frame_id = "tag_0";
            tf_msg.transform.translation.x = pos_filt.x;
            tf_msg.transform.translation.y = pos_filt.y;
            tf_msg.transform.translation.z = pos_filt.z;
            tf_msg.transform.rotation.x = q_filt.x();
            tf_msg.transform.rotation.y = q_filt.y();
            tf_msg.transform.rotation.z = q_filt.z();
            tf_msg.transform.rotation.w = q_filt.w();
            tf_broadcaster_->sendTransform(tf_msg);

            // --- Referencia frente al tag ---
            cv::Mat P_ref_tag = (cv::Mat1d(3,1) << 0.0, 0.3, 1.5);
            cv::Mat P_ref_cam = R_corrected * P_ref_tag + (cv::Mat1d(3,1) << pos_filt.x, pos_filt.y, pos_filt.z);

            geometry_msgs::msg::PoseStamped pose_cam;
            pose_cam.header.frame_id = "camera_gimbal";
            pose_cam.pose.position.x = P_ref_cam.at<double>(0);
            pose_cam.pose.position.y = P_ref_cam.at<double>(1);
            pose_cam.pose.position.z = P_ref_cam.at<double>(2);
            pose_cam.pose.orientation.w = 1.0;

            geometry_msgs::msg::PoseStamped pose_odom;
            try {
                pose_odom = tf_buffer_.transform(pose_cam, "odom", tf2::durationFromSec(0.05));
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
                continue;
            }

            reference_pub_->publish(pose_odom);

            std_msgs::msg::Float64MultiArray ref_vec;
            ref_vec.data = {
                pose_odom.pose.position.x,
                pose_odom.pose.position.y,
                pose_odom.pose.position.z,
                0.54,
                0.0, 0.0, 0.0, 0.0
            };
            ref_vec_pub_->publish(ref_vec);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose_odom.pose;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            marker_pub_->publish(marker);
        }

        std_msgs::msg::Bool det;
        det.data = tag_detected;
        detected_pub_->publish(det);
        apriltag_detections_destroy(detections);
    }

    // --- ROS y TF ---
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_vec_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    apriltag_family_t *tf_;
    apriltag_detector_t *td_;

    cv::Mat cameraMatrix_, distCoeffs_;
    bool has_camera_info_ = false;
    bool first_detection_;
    double tag_size_;
    double alpha_;
    cv::Point3d prev_pos_;
    tf2::Quaternion prev_q_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagTFDetector>());
    rclcpp::shutdown();
    return 0;
}
