#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "vissus_projekt/msg/odometry_array.hpp"

// msg typedef
typedef geometry_msgs::msg::Twist Msg_Twist;
typedef nav_msgs::msg::Odometry Msg_Odom;
typedef vissus_projekt::msg::OdometryArray Msg_Boit_Info;
typedef nav_msgs::msg::OccupancyGrid Msg_Map;
// pubs/subs typedef
typedef rclcpp::Publisher<Msg_Twist>::SharedPtr Publisher_Twist;
typedef rclcpp::Subscription<Msg_Odom>::SharedPtr Subscription_Odom;
typedef rclcpp::Subscription<Msg_Boit_Info>::SharedPtr Subscription_Boit_Info;
typedef rclcpp::Publisher<Msg_Boit_Info>::SharedPtr Publisher_Boit_Info;
typedef rclcpp::Subscription<Msg_Map>::SharedPtr Subscription_Map;
