#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "utils.h"
#include "defines.h"

using namespace std::chrono_literals;

Vector2 parseVector2(const std::string &input)
{
    std::string s = input;
    s.erase(std::remove(s.begin(), s.end(), '['), s.end());
    s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
    s.erase(std::remove(s.begin(), s.end(), ' '), s.end());

    Vector2 result;
    std::stringstream ss(s);
    std::string item;
    std::getline(ss, item, ',');
    if (!item.empty())
        result.x = std::stof(item);
    if (std::getline(ss, item, ',') && !item.empty())
        result.y = std::stof(item);
    return result;
}

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
        auto goal_str = this->declare_parameter<std::string>("goal", "[0.0, 0.0]");

        goal_ = parseVector2(goal_str);

        odom_msg_.header.frame_id = "world";
        odom_msg_.pose.pose.position.x = goal_.x;
        odom_msg_.pose.pose.position.y = goal_.y;
        odom_msg_.pose.pose.position.z = 0.0;

        publisher_ = this->create_publisher<Msg_Odom>("/goal_odom", 10);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), 
            "MinimalPublisher started. Publishing Odometry at (%.2f, %.2f)",
            goal_.x, goal_.y);
    }

private:
    void timer_callback()
    {
        odom_msg_.header.stamp = this->now();

        publisher_->publish(odom_msg_);

        RCLCPP_INFO(this->get_logger(),
            "Published Odometry: position=(%.2f, %.2f)",
            odom_msg_.pose.pose.position.x,
            odom_msg_.pose.pose.position.y);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Odometry odom_msg_;
    Vector2 goal_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
