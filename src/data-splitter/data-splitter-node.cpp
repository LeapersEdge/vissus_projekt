#pragma once

#include "defines.h"
#include "rclcpp/rclcpp.hpp"
#include "utils.h"

using std::placeholders::_1;

/* CODE SNIPPET FOR BACK ANGLE IGNORING

void Game::Apply_Avoidence(unsigned int boid_index)
{
    // sum of all avoidence forces that each boid in swarm applies to boid in boid_index
    Vector2 avoidence_strength = (Vector2){0.0f,0.0f};

    for (unsigned int i = 0; i < last_number_of_boids; i++)
    {
        if (i == boid_index)
            continue;

        // order is not inverse of this, vector direction is handeled by Boid::Update_Velocity
        Vector2 delta;
        delta.x = boids[i].position.x - boids[boid_index].position.x;
        delta.y = boids[i].position.y - boids[boid_index].position.y;


        // is within radius? 
        // is within not cropped out part of the circle relative to current direction?
        if ((delta.x * delta.x + delta.y * delta.y) <= (avoidence_visual_field_radius * avoidence_visual_field_radius) &&                                 
            std::abs(Vector2Angle(boids[boid_index].velocity, delta)) <= side_angel_view)  
        {
            avoidence_strength.x += delta.x / (delta.x * delta.x + delta.y * delta.y);
            avoidence_strength.y += delta.y / (delta.x * delta.x + delta.y * delta.y);
        }
    }

    boids[boid_index].acceleration_avoidence.x += avoidence_strength.x * avoidence_factor;
    boids[boid_index].acceleration_avoidence.y += avoidence_strength.y * avoidence_factor;
}


BITAN JE DIO
    std::abs(Vector2Angle(boids[boid_index].velocity, delta)) <= side_angel_vie
    float Vector2Angle(Vector2 v1, Vector2 v2)
    {
        float result = atan2f(v2.y - v1.y, v2.x - v1.x)*(180.0f/PI);

        if (result < 0) result += 360.0f;

        return result;
    }
TO SAM TAKO KORISTIO JER JE .VELOCITY IMAO INFORMATION I SMJERU, ALI SADA TO IMAMO DIREKTNO IZ KUTA,
DODUSE NEZ AKO BI NAM BILO MUDRIJE ISKORISTITI TO ILI OVO
*/

class Data_Splitter_Node : public rclcpp::Node
{
public:
    Data_Splitter_Node() : Node("data_splitter_node")
    {
        boit_fov_ = this->declare_parameter<float>("boit_fov", 0.0f);
        boit_vision_range_ = this->declare_parameter<float>("boit_vision_range", 0.0f);
        num_boids_ = this->declare_parameter<uint>("boid_number", 1);

        map_sub_ = this->create_subscription<Msg_Map>("/map", 10, std::bind(&Data_Splitter_Node::Map_Callback, this, _1));
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        
        uint[] robot_ids;
        for (int i = 0; i < num_boids_; ++i) {
            robot_ids.push_back(i);
        }

        // generiranje topica za subscribere
        std::vector<std::string> odom_topics;
        std::vector<std::string> cmd_vel_topics;

        for (uint id : robot_ids) {
            odom_topics.push_back("/" + std::to_string(id) + "/odom");
            cmd_vel_topics.push_back("/" + std::to_string(id) + "/cmd_vel");
        }

        for (uint i = 0; i < highest_robot_id; ++i)
        {
            Subscription_Odom sub = this->create_subscription<Msg_Odom>(
                    odom_topics[i], 
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
    
    Subscription_Map map_sub_;

    float boit_fov_;
    float boit_vision_range_;
    float num_boids_;

    std::vector<Publisher_Twist> pubs;
    std::vector<Subscription_Odom> subs;
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Data_Splitter_Node::odom_msg_callback, this));

    std::vector<Publisher_Twist> pubs;
    std::vector<Subscription_Odom> subs;

    Msg_Map map_;

    // publisher koji ce publishati poruke o obstaclima i susjedima
    void Subscription_Odom_Callback(const Msg_Odom::SharedPtr odom, uint id){
        
    }
    
    void Map_Callback(const Msg_Map::SharedPtr map){
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
