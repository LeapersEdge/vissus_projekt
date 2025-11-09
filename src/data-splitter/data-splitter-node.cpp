#include "defines.h"
#include "rclcpp/rclcpp.hpp"
#include "utils.h"
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include "utils.h"
#include "vector2.hpp"

#define _USE_MATH_DEFINES

using std::placeholders::_1;
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
float normalize_angle(float angle);

/**
 * @class Data_Splitter_Node
 * @brief ROS 2 node that collects odometry data from multiple robots and publishes 
 *        Boid information as OddometryArray msg. This includes neighbours and closest obstacles.
 */
class Data_Splitter_Node : public rclcpp::Node
{
public:
    Data_Splitter_Node() : Node("data_splitter_node")
    {
        //ros parameters of node
        boid_fov_ = this->declare_parameter<float>("boid_fov", M_PI);
        boid_vision_range_ = this->declare_parameter<float>("boid_vision_range", 10.0f);
        num_boids_ = (unsigned int)this->declare_parameter<int>("boid_number", 1u);

        map_sub_ = this->create_subscription<Msg_Map>("/map", 10, std::bind(&Data_Splitter_Node::Map_Callback, this, _1));

        // construction of robot ids and topic array: /robot_<id>/odom, /robot_<id>/boit_info
        // these topic will be used by publishers
        for (unsigned int i = 0; i < num_boids_; ++i) {
            robot_ids_.push_back(i);
        }
        std::vector<std::string> odom_topics;
        std::vector<std::string> cmd_vel_topics;
        for (unsigned int id : robot_ids_) {
            odom_topics.push_back("/robot_" + std::to_string(id) + "/odom");
            cmd_vel_topics.push_back("/robot_" + std::to_string(id) + "/boit_info");
        }

        robot_odoms_.resize(num_boids_);

        // just declarations of publishers and suscriber 
        for (unsigned int i = 0; i < num_boids_; ++i)
        {
            Subscription_Odom sub = this->create_subscription<Msg_Odom>(
                    odom_topics[i], 
                    10, 
                    [this, i](const Msg_Odom::SharedPtr msg) {
                        this->Subscription_Odom_Callback(msg, i);
                    }
                ); 
            subs_odom_.push_back(sub);
            
            Publisher_Boit_Info pub = this->create_publisher<Msg_Boit_Info>(
                    cmd_vel_topics[i], 
                    10
                ); 
            pubs_boit_info_.push_back(pub);
        }
    }

private:
    Subscription_Map map_sub_;
    std::vector<Publisher_Boit_Info> pubs_boit_info_; //OdometryArray msgs spublishers
    std::vector<Subscription_Odom> subs_odom_;
    Msg_Map map_;

    float boid_fov_; // field of view
    float boid_vision_range_;  // how far can boid see obstacles and other boids(neighbours)
    unsigned int num_boids_; 
    std::vector<unsigned int> robot_ids_; //array of all robot ids

    std::vector<Msg_Odom> robot_odoms_;

    /**
    * @brief Computes the closest obstacle to a given robot odometry.
    * 
    * This function searches through the occupancy grid to find the nearest
    * occupied cell (obstacle) within the robot's vision range.
    * 
    * @param robot_odom The odometry of the robot.
    * @return Vector2 The coordinates of the closest obstacle.
    */
    Vector2 get_closest_obstacle(const Msg_Odom& robot_odom){
        unsigned int W = map_.info.width;
        unsigned int H = map_.info.height;
        float resolution = map_.info.resolution;
        float origin_x = map_.info.origin.position.x;
        float origin_y = map_.info.origin.position.y;

        float x_odom = robot_odom.pose.pose.position.x;
        float y_odom = robot_odom.pose.pose.position.y;
        Vector2 pose_odom{x_odom, y_odom};

        float smallest_distance = std::numeric_limits<float>::max();
        Vector2 closest_obstacle{0.0f, 0.0f};

        for (unsigned int i = 0; i < H; i++) {
            for (unsigned int j = 0; j < W; j++) {
                size_t idx = static_cast<size_t>(i) * static_cast<size_t>(W) + static_cast<size_t>(j);
                if (idx >= map_.data.size()) continue;
                if (map_.data[idx] <= 0) continue;

                float cell_x = origin_x + j * resolution + resolution / 2.0f;
                float cell_y = origin_y + i * resolution + resolution / 2.0f;
                Vector2 cell_pos{cell_x, cell_y};

                float dx = cell_pos.x - pose_odom.x;
                float dy = cell_pos.y - pose_odom.y;
                Vector2 cell_i{dx, dy};
                float distance_sq = squared_euclidan_norm(cell_i);

                if (distance_sq < boid_vision_range_ * boid_vision_range_ &&
                        distance_sq < smallest_distance) 
                {
                    smallest_distance = distance_sq;
                    closest_obstacle = cell_pos;
                }
            }
        }
        return closest_obstacle;
    }
    /**
    * @brief Computes the closest obstacle to a given robot odometry.
    * 
    * This function searches through the occupancy grid to find the nearest
    * occupied cell (obstacle) within the robot's vision range.
    *
    * @param robot_odoms Array of all recorded odometries
    * @return Vector2 The coordinate of the closest obstacle.
    */
    std::vector<Msg_Odom> get_neighbours(const std::vector<Msg_Odom>& robot_odoms, unsigned int id){
        if (id >= robot_odoms.size()) return {};
        const Msg_Odom& boid_id_odom = robot_odoms[id];
        Vector2 boid_pose{
            (float)boid_id_odom.pose.pose.position.x, 
            (float)boid_id_odom.pose.pose.position.y
        };
        float yaw = getYawFromQuaternion(boid_id_odom.pose.pose.orientation); //from rotation quaternion mathematical definition

        std::vector<Msg_Odom> true_neighbours;

        for (size_t i = 0; i < robot_odoms.size(); ++i){
            if (static_cast<unsigned int>(i) == id) continue;   
            const Msg_Odom& neighbour_i_odom = robot_odoms[i];
            Vector2 neighbour_i{
                (float)neighbour_i_odom.pose.pose.position.x, 
                (float)neighbour_i_odom.pose.pose.position.y
            };
            Vector2 diff{neighbour_i.x - boid_pose.x, neighbour_i.y - boid_pose.y};
            float distance_sq = squared_euclidan_norm(diff);
            float angle = atan2(diff.x, diff.y);

            float relative_angle = normalize_angle(angle - yaw);

            //if (distance_sq < boid_vision_range_ * boid_vision_range_ &&
            //    ((relative_angle < (boid_fov_/2.0f)) && (relative_angle > (-boid_fov_/2.0f)))) {
            //    true_neighbours.push_back(neighbour_i_odom);
            //}
            true_neighbours.push_back(neighbour_i_odom);
            
        }
        return true_neighbours;
    }
    /**
    * @brief Computes the closest obstacle to a given robot odometry.
    * 
    * This callback is called by boids/odom sub and constructs OdometryArray(boit_info)
    * message.
    * 
    * @param odom Pointer to the robot odometry 
    * @return unsigned_int id Robot id
    */
    void Subscription_Odom_Callback(const Msg_Odom::SharedPtr odom, unsigned int id){
        if (id >= robot_odoms_.size()) return;
        robot_odoms_[id] = *odom;
        std::vector<Msg_Odom> neighbours = get_neighbours(robot_odoms_, id); 
        Msg_Boit_Info odom_array_id;

        odom_array_id.odometries = neighbours;
        odom_array_id.odometries.insert(odom_array_id.odometries.begin(), *odom);
        Vector2 closest = get_closest_obstacle(*odom);
        //potrebno je pretvoriti u double jer Point message koristi double
        odom_array_id.closest_obstacle.x = static_cast<double>(closest.x);
        odom_array_id.closest_obstacle.y = static_cast<double>(closest.y);
        odom_array_id.closest_obstacle.z = 0.0;

        pubs_boit_info_[id]->publish(odom_array_id);
    }

    // saves map as private variable
    void Map_Callback(const Msg_Map::SharedPtr map){
        map_ = *map;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Data_Splitter_Node>());
    rclcpp::shutdown();
    return 0;
}       

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    double x = q.x;
    double y = q.y;
    double z = q.z;
    double w = q.w;

    double yaw = std::atan2(2.0 * (w * z + x * y),
                            1.0 - 2.0 * (y * y + z * z));

    return yaw;
}

float normalize_angle(float angle)
{
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}
