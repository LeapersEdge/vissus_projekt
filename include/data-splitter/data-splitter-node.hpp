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
        map_sub_ = this->create_subscription<Msg_Map>("/map", 10, std::bind(&Data_Splitter_Node::Map_Callback, this))

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
    
    rclcpp::Subscription<Msg_Map> map_sub_;

    float boit_fov_;
    float boit_vision_range_;
    
    std::vector<Publisher_Twist> pubs;
    std::vector<Subscription_Odom> subs;

    Msg_Map map_;

    void Subscription_Odom_Callback(const Msg_Odom::SharedPtr odom, uint id);
    void Data_Splitter_Node::Map_Callback(const Msg_Map::SharedPtr map){
        map_ = *map;
    }

    Vector2 get_closest_obstacle(const Msg_Odom& robot_odom) {
        if (map_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Map is empty!");
            return Vector2{0.0f, 0.0f};
        }

        uint W = map_.info.width;
        uint H = map_.info.height;
        float resolution = map_.info.resolution;
        float origin_x = map_.info.origin.position.x;
        float origin_y = map_.info.origin.position.y;

        // Robot position in map frame
        float x_odom = robot_odom.pose.pose.position.x;
        float y_odom = robot_odom.pose.pose.position.y;
        Vector2 pose_odom{x_odom, y_odom};

        float smallest_distance = std::numeric_limits<float>::max();
        Vector2 closest_obstacle{0.0f, 0.0f};

        for (size_t i = 0; i < H; i++) {
            for (size_t j = 0; j < W; j++) {
                size_t idx = i * W + j;
                if (map_.data[idx] <= 0) continue;

                float cell_x = origin_x + j * resolution + resolution / 2.0f;
                float cell_y = origin_y + i * resolution + resolution / 2.0f;
                Vector2 cell_pos{cell_x, cell_y};

                float dx = cell_pos.x - pose_odom.x;
                float dy = cell_pos.y - pose_odom.y;
                Vector2 cell_i{dx, dy};
                float distance_sq = squared_euclidan_norm(cell_i - pose_odom);

                if (distance_sq < boit_vision_range_ * boit_vision_range_ &&
                    distance_sq < smallest_distance) 
                {
                    smallest_distance = distance_sq;
                    closest_obstacle = cell_pos;
                }
            }
        }

        return closest_obstacle;
    }


};
