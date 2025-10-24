#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/opencv.hpp>
#include <chrono>

// -----------------------
// Filtro de Kalman para posición
// -----------------------
class KalmanFilter3D {
public:
    KalmanFilter3D(double dt = 0.033) {
        kf_ = cv::KalmanFilter(6, 3, 0);
        F_ = (cv::Mat_<float>(6,6) <<
            1,0,0,dt,0,0,
            0,1,0,0,dt,0,
            0,0,1,0,0,dt,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1);
        H_ = (cv::Mat_<float>(3,6) <<
            1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,1,0,0,0);

        kf_.transitionMatrix = F_;
        kf_.measurementMatrix = H_;
        setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-4));
        setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-2));
        setIdentity(kf_.errorCovPost, cv::Scalar::all(1));
        kf_.statePost = cv::Mat::zeros(6, 1, CV_32F);
    }

    cv::Point3f update(const cv::Point3f& meas) {
        cv::Mat measurement(3, 1, CV_32F);
        measurement.at<float>(0) = meas.x;
        measurement.at<float>(1) = meas.y;
        measurement.at<float>(2) = meas.z;

        kf_.predict();
        cv::Mat estimated = kf_.correct(measurement);

        return cv::Point3f(
            estimated.at<float>(0),
            estimated.at<float>(1),
            estimated.at<float>(2)
        );
    }

private:
    cv::KalmanFilter kf_;
    cv::Mat F_, H_;
};

// -----------------------
// Filtro para orientación (SLERP)
// -----------------------
class QuaternionFilter {
public:
    QuaternionFilter(double alpha = 0.2)
        : alpha_(alpha), initialized_(false) {}

    tf2::Quaternion update(const tf2::Quaternion& q_meas) {
        if (!initialized_) {
            q_filtered_ = q_meas;
            initialized_ = true;
            return q_filtered_;
        }
        q_filtered_ = q_filtered_.slerp(q_meas, alpha_);
        q_filtered_.normalize();
        return q_filtered_;
    }

private:
    tf2::Quaternion q_filtered_;
    bool initialized_;
    double alpha_;
};

// -----------------------
// Nodo principal ROS2
// -----------------------
class KalmanFilterPoseNode : public rclcpp::Node {
public:
    KalmanFilterPoseNode()
        : Node("KalmanNode"),
          buffer_(this->get_clock()),
          listener_(buffer_),
          kf_pos_(0.033),
          q_filter_(0.2)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Parámetros ROS2 configurables
        this->declare_parameter<std::string>("parent_frame", "camera_link");
        this->declare_parameter<std::string>("child_frame", "tag_0");
        this->declare_parameter<std::string>("filtered_frame", "Tag_0_filtered");
        this->declare_parameter<double>("dt", 0.033);
        this->declare_parameter<double>("alpha", 0.2);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&KalmanFilterPoseNode::process, this)
        );

        RCLCPP_INFO(this->get_logger(),
                    "Kalman filter pose node started. Listening for TF transforms.");
    }

private:
    void process() {
        std::string parent_frame = this->get_parameter("parent_frame").as_string();
        std::string child_frame = this->get_parameter("child_frame").as_string();
        std::string filtered_frame = this->get_parameter("filtered_frame").as_string();

        geometry_msgs::msg::TransformStamped tf_raw;
        try {
            tf_raw = buffer_.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            return;
        }

        // --- Filtrar posición ---
        cv::Point3f meas(
            tf_raw.transform.translation.x,
            tf_raw.transform.translation.y,
            tf_raw.transform.translation.z
        );
        cv::Point3f filtered_pos = kf_pos_.update(meas);

        // --- Filtrar orientación ---
        tf2::Quaternion q_meas(
            tf_raw.transform.rotation.x,
            tf_raw.transform.rotation.y,
            tf_raw.transform.rotation.z,
            tf_raw.transform.rotation.w
        );
        tf2::Quaternion q_filtered = q_filter_.update(q_meas);

        // --- Publicar TF filtrado ---
        geometry_msgs::msg::TransformStamped tf_filtered;
        tf_filtered.header.stamp = this->now();
        tf_filtered.header.frame_id = parent_frame;
        tf_filtered.child_frame_id = filtered_frame;
        tf_filtered.transform.translation.x = filtered_pos.x;
        tf_filtered.transform.translation.y = filtered_pos.y;
        tf_filtered.transform.translation.z = filtered_pos.z;
        tf_filtered.transform.rotation.x = q_filtered.x();
        tf_filtered.transform.rotation.y = q_filtered.y();
        tf_filtered.transform.rotation.z = q_filtered.z();
        tf_filtered.transform.rotation.w = q_filtered.w();

        tf_broadcaster_->sendTransform(tf_filtered);

        
    }

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    KalmanFilter3D kf_pos_;
    QuaternionFilter q_filter_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// -----------------------
// MAIN
// -----------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterPoseNode>());
    rclcpp::shutdown();
    return 0;
}
