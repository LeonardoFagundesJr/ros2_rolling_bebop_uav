#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <chrono>
#include <thread>

/*
Nodo ROS2 que envía un comando de aterrizaje (/bebop/land)
y espera 5 segundos antes de salir.

Autor: Brayan Saldarriaga-Mesa
*/

using namespace std::chrono_literals;

class LandNode : public rclcpp::Node
{
public:
    LandNode()
        : Node("land_node")
    {
        pub_land_ = this->create_publisher<std_msgs::msg::Empty>("/bebop/land", 10);
        RCLCPP_INFO(this->get_logger(), "Nodo de aterrizaje iniciado.");
        this->land();
    }

private:
    void land()
    {
        std_msgs::msg::Empty msg;
        pub_land_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LandNode>();
    rclcpp::spin(node);
    return 0;
}
