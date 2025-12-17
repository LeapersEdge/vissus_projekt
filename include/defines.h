#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "vissus_projekt/msg/odometry_array.hpp"
#include "vissus_projekt/msg/tuning_params.hpp"

// msg typedef
typedef geometry_msgs::msg::Twist Msg_Twist;
typedef nav_msgs::msg::Odometry Msg_Odom;
typedef vissus_projekt::msg::OdometryArray Msg_Boid_Info;
typedef nav_msgs::msg::OccupancyGrid Msg_Map;
typedef geometry_msgs::msg::Point Msg_Point;
typedef vissus_projekt::msg::TuningParams Msg_Tuning_Params; 
typedef geometry_msgs::msg::PoseStamped Msg_PoseStamped;

// pubs typedef
typedef rclcpp::Publisher<Msg_Twist>::SharedPtr Publisher_Twist;
typedef rclcpp::Publisher<Msg_Boid_Info>::SharedPtr Publisher_Boid_Info;
typedef rclcpp::Publisher<Msg_PoseStamped>::SharedPtr Publisher_PoseStamped;

// subs typedef
typedef rclcpp::Subscription<Msg_Boid_Info>::SharedPtr Subscription_Boid_Info;
typedef rclcpp::Subscription<Msg_Odom>::SharedPtr Subscription_Odom;
typedef rclcpp::Subscription<Msg_Map>::SharedPtr Subscription_Map;
typedef rclcpp::Subscription<Msg_Tuning_Params>::SharedPtr Subscription_Tuning_Params;
typedef rclcpp::Subscription<Msg_Point>::SharedPtr Subscription_Point;
typedef rclcpp::Subscription<Msg_PoseStamped>::SharedPtr Subscription_PoseStamped;
typedef rclcpp::Subscription<Msg_Twist>::SharedPtr Subscription_Twist;

