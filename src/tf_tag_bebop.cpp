// TagTFDetector (publishes tag position in odom frame)
// Author: Brayan Saldarriaga-Mesa (modified version)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
// #include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <thread>
#include <mutex>
#include <chrono>

class TagTFDetector : public rclcpp::Node {
public:
    TagTFDetector() :
        Node("tag_tf_detector"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_),
        tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
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

        sub_start_ = this->create_subscription<std_msgs::msg::Empty>(
            "/bebop/start_sequence", 10,
            std::bind(&TagTFDetector::start_callback, this, std::placeholders::_1));

        reference_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tag_reference_odom", 10);
        detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/bebop/detected", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("tag_marker", 10);
        pub_cam_ = this->create_publisher<geometry_msgs::msg::Vector3>("/bebop/move_camera", 10);
        pub_land_ = this->create_publisher<std_msgs::msg::Empty>("/bebop/land", 10);

        tag_size_ = 0.145;
        ref_y_ = -0.9;

        RCLCPP_INFO(this->get_logger(), "TagTFDetector initialized (publishing in odom frame).");
    }

    ~TagTFDetector() {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
    }

private:
    // --- CAMERA INFO CALLBACK ---
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

    // --- IMAGE CALLBACK ---
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
                {(float)-s,(float)-s,0}, 
                {(float)s,(float)-s,0}, 
                {(float)s,(float)s,0}, 
                {(float)-s,(float)s,0}
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

            // --- Orientation correction ---
            cv::Mat R_y180 = (cv::Mat1d(3,3) << -1,0,0, 0,1,0, 0,0,-1);
            cv::Mat R_z180 = (cv::Mat1d(3,3) << -1,0,0, 0,-1,0, 0,0,1);
            cv::Mat R_fix  = (cv::Mat1d(3,3) << 1,0,0, 0,-1,0, 0,0,-1);
            cv::Mat R_corrected = R_fix * R_z180 * R_y180 * R_tag2cam;

            tf2::Matrix3x3 tf2_rot(
                R_corrected.at<double>(0,0), R_corrected.at<double>(0,1), R_corrected.at<double>(0,2),
                R_corrected.at<double>(1,0), R_corrected.at<double>(1,1), R_corrected.at<double>(1,2),
                R_corrected.at<double>(2,0), R_corrected.at<double>(2,1), R_corrected.at<double>(2,2)
            );
            tf2::Quaternion q_tag;
            tf2_rot.getRotation(q_tag);
            q_tag.normalize();

            // --- TF tag relative to camera_link ---
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = this->now();
            tf_msg.header.frame_id = "camera_link";
            tf_msg.child_frame_id = "tag_0";
            tf_msg.transform.translation.x = tvec.at<double>(0);
            tf_msg.transform.translation.y = tvec.at<double>(1);
            tf_msg.transform.translation.z = tvec.at<double>(2);
            tf_msg.transform.rotation.x = q_tag.x();
            tf_msg.transform.rotation.y = q_tag.y();
            tf_msg.transform.rotation.z = q_tag.z();
            tf_msg.transform.rotation.w = q_tag.w();
            tf_broadcaster_->sendTransform(tf_msg);

            // --- RViz marker ---
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "camera_link";
            marker.header.stamp = this->now();
            marker.ns = "tag_marker";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = tvec.at<double>(0);
            marker.pose.position.y = tvec.at<double>(1);
            marker.pose.position.z = tvec.at<double>(2);
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 1.5;
            marker.color.r = 0.0f;
            marker.color.g = 0.5f;
            marker.color.b = 1.0f;
            marker.color.a = 0.6f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_pub_->publish(marker);

            // --- Reference in camera_link ---
            double ref_y_copy;
            { std::lock_guard<std::mutex> lock(ref_mutex_); ref_y_copy = ref_y_; }

            cv::Mat P_ref_tag = (cv::Mat1d(3,1) << 0.0, ref_y_copy, 1.0);
            cv::Mat P_ref_cam = R_corrected * P_ref_tag + tvec;

            geometry_msgs::msg::PoseStamped pose_cam;
            pose_cam.header.frame_id = "camera_link";
            pose_cam.header.stamp = this->now();
            pose_cam.pose.position.x = P_ref_cam.at<double>(0);
            pose_cam.pose.position.y = P_ref_cam.at<double>(1);
            pose_cam.pose.position.z = P_ref_cam.at<double>(2);
            pose_cam.pose.orientation.w = 1.0;

            // --- Transform pose_cam to odom frame ---
            try {
                geometry_msgs::msg::TransformStamped tf_cam_to_odom =
                    tf_buffer_.lookupTransform("odom", "camera_link", tf2::TimePointZero);
                
                geometry_msgs::msg::PoseStamped pose_odom;
                tf2::doTransform(pose_cam, pose_odom, tf_cam_to_odom);
                pose_odom.header.frame_id = "odom";
                reference_pub_->publish(pose_odom);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform odom<-camera_link unavailable: %s", ex.what());
            }
        }

        std_msgs::msg::Bool det;
        det.data = tag_detected;
        detected_pub_->publish(det);
        apriltag_detections_destroy(detections);
    }

    // --- SEQUENCE CALLBACK ---
    void start_callback(const std_msgs::msg::Empty::SharedPtr) {
        if (sequence_active_) return;
        sequence_active_ = true;
        std::thread(&TagTFDetector::run_sequence, this).detach();
    }

    void run_sequence() {
        using namespace std::chrono_literals;
        auto sleep = [](double s){ std::this_thread::sleep_for(std::chrono::duration<double>(s)); };

        geometry_msgs::msg::Vector3 cam_msg;
        std_msgs::msg::Empty land_msg;

        { std::lock_guard<std::mutex> lock(ref_mutex_); ref_y_ = -0.6; }
        RCLCPP_INFO(this->get_logger(), "Ref -0.6");
        sleep(3.0);

        cam_msg.x = -60.0; cam_msg.y = cam_msg.z = 0.0;
        pub_cam_->publish(cam_msg);
        RCLCPP_INFO(this->get_logger(), "Cam -60");
        sleep(2.0);

        { std::lock_guard<std::mutex> lock(ref_mutex_); ref_y_ = -0.4; }
        RCLCPP_INFO(this->get_logger(), "Ref -0.4");
        sleep(3.0);

        cam_msg.x = -75.0; pub_cam_->publish(cam_msg);
        RCLCPP_INFO(this->get_logger(), "Cam -75");
        sleep(3.0);

        { std::lock_guard<std::mutex> lock(ref_mutex_); ref_y_ = -0.2; }
        RCLCPP_INFO(this->get_logger(), "Ref -0.2");
        sleep(3.0);

        cam_msg.x = -90.0; pub_cam_->publish(cam_msg);
        RCLCPP_INFO(this->get_logger(), "Cam -90");
        sleep(3.0);

        { std::lock_guard<std::mutex> lock(ref_mutex_); ref_y_ = -0.1; }
        RCLCPP_INFO(this->get_logger(), "Ref 0.0");
        sleep(5.0);

        pub_land_->publish(land_msg);
        RCLCPP_INFO(this->get_logger(), "Land 1");
        std::this_thread::sleep_for(200ms);
        pub_land_->publish(land_msg);
        RCLCPP_INFO(this->get_logger(), "Land 2");

        sequence_active_ = false;
    }

    // --- ROS & TF ---
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_start_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_cam_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    apriltag_family_t *tf_;
    apriltag_detector_t *td_;
    cv::Mat cameraMatrix_, distCoeffs_;
    bool has_camera_info_ = false;
    double tag_size_;

    // --- Sequence control ---
    std::atomic<bool> sequence_active_{false};
    double ref_y_;
    std::mutex ref_mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagTFDetector>());
    rclcpp::shutdown();
    return 0;
}
