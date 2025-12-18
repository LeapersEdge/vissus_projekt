#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "vissus_projekt/msg/odometry_array.hpp"
#include "vissus_projekt/msg/tuning_params.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include <vector>

// msg typedef
typedef geometry_msgs::msg::Twist Msg_Twist;
typedef nav_msgs::msg::Odometry Msg_Odom;
typedef vissus_projekt::msg::OdometryArray Msg_Boid_Info;
typedef nav_msgs::msg::OccupancyGrid Msg_Map;
typedef geometry_msgs::msg::Point Msg_Point;
typedef vissus_projekt::msg::TuningParams Msg_Tuning_Params; 
typedef geometry_msgs::msg::PoseStamped Msg_PoseStamped;
typedef crazyflie_interfaces::msg::LogDataGeneric Msg_LogDataGeneric;

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
typedef rclcpp::Subscription<Msg_LogDataGeneric>::SharedPtr Subscription_LogDataGeneric;

typedef message_filters::Subscriber<Msg_Twist> Sub_Filter_Twist;
typedef message_filters::Subscriber<Msg_PoseStamped> Sub_Filter_PoseStamped;
typedef message_filters::Subscriber<Msg_LogDataGeneric> Sub_Filter_LogDataGeneric;

typedef message_filters::sync_policies::ApproximateTime<Msg_PoseStamped, Msg_LogDataGeneric> SyncPolicy;

typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
typedef std::vector<std::vector<bool>> Bool_mat;

