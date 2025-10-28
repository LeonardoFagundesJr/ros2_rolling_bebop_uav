// TagTFDetector: ROS2 node that detects AprilTags using OpenCV and AprilTag 3,
// estimates the 3D pose of tag_0 relative to the gimbal-mounted camera (camera_gimbal frame),
// and publishes both PoseStamped and TF transforms, as well as a boolean /bebop/detected indicating detection status.
// Author: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar), in collaboration with UFV.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

class TagTFDetector : public rclcpp::Node {
public:
    TagTFDetector() : Node("tag_tf_detector") {
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
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/bebop/detected", 10);

        // Parámetros
        tag_size_ = 0.145;   // metros
        alpha_ = 0.85;       // suavizado temporal
        first_detection_ = true;

        // Configuración del detector
        td_->quad_decimate = 2.0;
        td_->quad_sigma = 0.8;
        td_->refine_edges = 1;

        RCLCPP_INFO(this->get_logger(), "TagTFDetector initialized (alpha=%.2f)", alpha_);
    }

    ~TagTFDetector() {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
    }

private:
    // --- Callback de cámara ---
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

    // --- Callback de imagen ---
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
        cv::GaussianBlur(gray, gray, cv::Size(3,3), 0);

        image_u8_t im = { gray.cols, gray.rows, gray.cols, gray.data };
        zarray_t* detections = apriltag_detector_detect(td_, &im);

        bool tag_detected = false;

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);
            if (det->id != 0) continue;  // solo el tag_0
            tag_detected = true;

            // --- Estimar pose ---
            double s = tag_size_ / 2.0;
            std::vector<cv::Point3f> objectPoints = {
                {-s, -s, 0}, { s, -s, 0}, { s,  s, 0}, { -s,  s, 0}
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

            // --- Rotaciones personalizadas (manteniendo tu lógica original) ---
            cv::Mat R_fix = (cv::Mat1d(3,3) <<
                 1,  0,  0,
                 0, -1,  0,
                 0,  0, -1
            );
            cv::Mat R_corrected = R_fix * R_tag2cam;

            cv::Mat R_flipX = (cv::Mat1d(3,3) <<
                 1,  0,  0,
                 0, -1,  0,
                 0,  0, -1
            );
            R_corrected = R_flipX * R_corrected;

            // --- Convertir a cuaternión ---
            tf2::Matrix3x3 tf2_rot(
                R_corrected.at<double>(0,0), R_corrected.at<double>(0,1), R_corrected.at<double>(0,2),
                R_corrected.at<double>(1,0), R_corrected.at<double>(1,1), R_corrected.at<double>(1,2),
                R_corrected.at<double>(2,0), R_corrected.at<double>(2,1), R_corrected.at<double>(2,2)
            );
            tf2::Quaternion q_curr;
            tf2_rot.getRotation(q_curr);
            q_curr.normalize();

            cv::Point3d pos_curr(
                tvec.at<double>(0),
                tvec.at<double>(1),
                tvec.at<double>(2)
            );

            // --- Filtro exponencial (posición + orientación) ---
            if (first_detection_) {
                prev_pos_ = pos_curr;
                prev_q_ = q_curr;
                first_detection_ = false;
            }

            cv::Point3d pos_filt = alpha_ * prev_pos_ + (1 - alpha_) * pos_curr;
            tf2::Quaternion q_filt = prev_q_.slerp(q_curr, 1.0 - alpha_);
            q_filt.normalize();

            prev_pos_ = pos_filt;
            prev_q_ = q_filt;

            // --- Publicar Pose ---
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

            // --- Publicar TF ---
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
        }

        // --- Publicar detección ---
        std_msgs::msg::Bool detect_msg;
        detect_msg.data = tag_detected;
        detected_pub_->publish(detect_msg);

        apriltag_detections_destroy(detections);
    }

    // --- ROS ---
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // --- AprilTag ---
    apriltag_family_t *tf_;
    apriltag_detector_t *td_;

    // --- Cámara ---
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    bool has_camera_info_ = false;

    // --- Filtro ---
    double tag_size_;
    double alpha_;
    bool first_detection_;
    cv::Point3d prev_pos_;
    tf2::Quaternion prev_q_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagTFDetector>());
    rclcpp::shutdown();
    return 0;
}
