#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

// msg typedef
typedef geometry_msgs::msg::Twist Msg_Twist;
typedef nav_msgs::msg::Odometry Msg_Odom;
// pubs/subs typedef
typedef rclcpp::Publisher<Msg_Twist>::SharedPtr Publisher_Twist;
typedef rclcpp::Subscription<Msg_Odom>::SharedPtr Subscription_Odom;
