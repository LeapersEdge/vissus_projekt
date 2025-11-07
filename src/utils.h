#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#pragma once

// msg typedef
typedef geometry_msgs::msg::Twist Msg_Twist;
typedef nav_msgs::msg::Odometry Msg_Odom;
// pubs/subs typedef
typedef rclcpp::Publisher<Msg_Twist>::SharedPtr Publisher_Twist;
typedef rclcpp::Subscription<Msg_Odom>::SharedPtr Subscription_Odom;


struct Vector2
{
    float x = 0.0f;
    float y = 0.0f;
};

struct Boit
{
    Msg_Odom odom;
    bool initialized = false;
    float last_rotation = 0.0f; 
    clock_t last_time = 0;
};

float squared_euclidan_norm(Vector2 vec);

float p_controller_update(float reference, float state, float kp);

bool string_contains(std::string string, std::string substring);

uint parse_number_from_string(std::string string);