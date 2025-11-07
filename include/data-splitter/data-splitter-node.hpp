#pragma once

#include "rclcpp/rclcpp.hpp"
#include "utils.h"

using std::placeholders::_1;

class Data_Splitter_Node : public rclcpp::Node
{
public:
    Data_Splitter_Node() : Node("data_splitter_node")
    {
        boit_fov_ = this->declare_parameter<float>("boit_fov", 0.0f);
        boit_vision_range_ = this->declare_parameter<float>("boit_vision_range", 0.0f);

        uint highest_robot_id = 0;
        {
            auto topics = get_topic_names_and_types();
            for (const auto& topic : topics)
            {
                std::string topic_name = topic.first;
                if (string_contains(topic_name, "robot_") && string_contains(topic_name, "/odom"))
                {
                    uint robot_id = parse_number_from_string(topic_name);
                    if (robot_id > highest_robot_id)
                        highest_robot_id = robot_id;                
                }
            }
        }
        
        for (uint i = 0; i < highest_robot_id; ++i)
        {
            std::string robot_odom_topic = "/robot_" + std::to_string(i) + "/odom";
            std::string robot_twist_topic = "/robot_" + std::to_string(i) + "/cmd_vel";
        
            Subscription_Odom sub = this->create_subscription<Msg_Odom>(
                    robot_odom_topic, 
                    10, 
                    [this, i](const Msg_Odom::SharedPtr msg) {
                        this->Subscription_Odom_Callback(msg, i);
                    }
                ); 
            subs.push_back(sub);
            
            // init pub
            Publisher_Twist pub = this->create_publisher<Msg_Twist>(
                    robot_twist_topic, 
                    10
                ); 
            pubs.push_back(pub);
        }
    }
private:
    void Subscription_Odom_Callback(const Msg_Odom::SharedPtr odom, uint id);
private:
    float boit_fov_;
    float boit_vision_range_;
    
    std::vector<Publisher_Twist> pubs;
    std::vector<Subscription_Odom> subs;
};
