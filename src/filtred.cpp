#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <deque>

class RefVecFilter : public rclcpp::Node {
public:
    RefVecFilter() :
        Node("ref_vec_filter"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        sub_ref_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "tag_reference_odom", 10,
            std::bind(&RefVecFilter::ref_callback, this, std::placeholders::_1));

        ref_vec_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bebop/ref_vec", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("ref_vec_marker", 10);

        alpha_ = this->declare_parameter("alpha", 0.9);           // filtro exponencial
        window_size_ = this->declare_parameter("window_size", 20); // media móvil
        initialized_ = false;

        RCLCPP_INFO(this->get_logger(), "RefVecFilter initialized (α=%.2f, window=%d)", alpha_, window_size_);
    }

private:
    void add_sample(double x, double y) {
        x_hist_.push_back(x);
        y_hist_.push_back(y);
        if ((int)x_hist_.size() > window_size_) {
            x_hist_.pop_front();
            y_hist_.pop_front();
        }
    }

    std::pair<double,double> get_mean() {
        double sx = 0, sy = 0;
        for (size_t i = 0; i < x_hist_.size(); i++) {
            sx += x_hist_[i];
            sy += y_hist_[i];
        }
        return {sx / x_hist_.size(), sy / y_hist_.size()};
    }

    void ref_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose_odom;

        // --- Transform input to odom frame if needed ---
        try {
            if (msg->header.frame_id != "odom") {
                geometry_msgs::msg::TransformStamped tf_to_odom =
                    tf_buffer_.lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
                tf2::doTransform(*msg, pose_odom, tf_to_odom);
            } else {
                pose_odom = *msg;
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform to odom unavailable: %s", ex.what());
            return;
        }

        double x = pose_odom.pose.position.x;
        double y = pose_odom.pose.position.y;
        double z = 1.0;
        double yaw = 0.0;

        add_sample(x, y);
        auto [x_mean, y_mean] = get_mean();

        if (!initialized_) {
            x_f_ = x_mean;
            y_f_ = y_mean;
            initialized_ = true;
        } else {
            x_f_ = alpha_ * x_f_ + (1 - alpha_) * x_mean;
            y_f_ = alpha_ * y_f_ + (1 - alpha_) * y_mean;
        }

        // --- Publish filtered vector ---
        std_msgs::msg::Float64MultiArray ref_msg;
        ref_msg.data = {x_f_, y_f_, z, yaw, 0.0, 0.0, 0.0, 0.0};
        ref_vec_pub_->publish(ref_msg);

        // --- Publish RViz marker ---
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "ref_vec";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x_f_;
        marker.pose.position.y = y_f_;
        marker.pose.position.z = 1.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.3);
        marker_pub_->publish(marker);
    }

    // --- ROS ---
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ref_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_vec_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // --- TF ---
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // --- Filters ---
    std::deque<double> x_hist_, y_hist_;
    int window_size_;
    double alpha_;
    double x_f_, y_f_;
    bool initialized_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RefVecFilter>());
    rclcpp::shutdown();
    return 0;
}
