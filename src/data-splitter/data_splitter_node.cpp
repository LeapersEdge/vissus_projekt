#include "rclcpp/rclcpp.hpp"
#include "utils.h"
#include <vector>
#include <string>
#include <limits>
#include <cmath>
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
        std::string filepath = "/root/ros_ws/src/vissus_projekt/src/launch/directed_graph.config";
        //ros parameters of node
        boid_fov_ = this->declare_parameter<float>("boid_fov", M_PI);
        boid_vision_range_ = this->declare_parameter<float>("boid_vision_range", 10.0f);
        num_boids_ = static_cast<unsigned int>(this->declare_parameter<int>("boid_number", 1));

        for (unsigned int i = 0; i < num_boids_; ++i) {
            robot_cfIDs_.push_back(i);
        }

        std::vector<std::string> cfID_pose_topics;
        std::vector<std::string> cfID_vel_topics;
	std::vector<std::string> cfID_odom_topics;

        for (unsigned int id : robot_cfIDs_) {
            cfID_odom_topics.push_back("/robot_" + std::to_string(id) + "/odom");
            cfID_pose_topics.push_back("/cf_" + std::to_string(id) + "/pose");
            cfID_vel_topics.push_back("/cf_" + std::to_string(id) + "/velocity");
        }

        robot_odoms_.resize(num_boids_);

        for (unsigned int i = 0; i < num_boids_; ++i)
        {
            Subscription_PoseStamped sub_posestamped =
                this->create_subscription<Msg_PoseStamped>(
                    cfID_pose_topics[i],
                    10,
                    [this, i](const Msg_PoseStamped::SharedPtr msg) {
                        this->Subscription_cfID_PoseStamped_Callback(msg, i);
                    });

            subs_posestamped_.push_back(sub_posestamped);

            Subscription_LogDataGeneric sub_log_data =
                this->create_subscription<Msg_LogDataGeneric>(
                    cfID_vel_topics[i],
                    10,
                    [this, i](const Msg_LogDataGeneric::SharedPtr msg) {
                        this->Subscription_cfID_LogDataGeneric_Callback(msg, i);
                    });

            subs_log_data_generic_.push_back(sub_log_data);

            Publisher_Boid_Info pub_boidinfo =
                this->create_publisher<Msg_Boid_Info>(
                    cfID_odom_topics[i],
                    10);

            pubs_boid_info_.push_back(pub_boidinfo);
        }
    }

private:
    Subscription_Map map_sub_;
    std::vector<Publisher_Boid_Info> pubs_boid_info_;
    std::vector<Subscription_PoseStamped> subs_posestamped_;
    std::vector<Subscription_LogDataGeneric> subs_log_data_generic_;
    Msg_Map map_;

    float boid_fov_;
    float boid_vision_range_;
    unsigned int num_boids_;
    std::vector<unsigned int> robot_cfIDs_;

    std::vector<Msg_Odom> robot_odoms_;

    Vector2 get_closest_obstacle(const Msg_Odom& robot_odom)
    {
        unsigned int W = map_.info.width;
        unsigned int H = map_.info.height;
        float resolution = map_.info.resolution;
        float origin_x = map_.info.origin.position.x;
        float origin_y = map_.info.origin.position.y;

        Vector2 pose_odom{
            robot_odom.pose.pose.position.x,
            robot_odom.pose.pose.position.y};

        float smallest_distance = std::numeric_limits<float>::max();
        Vector2 closest_obstacle{0.0f, 0.0f};

        for (unsigned int i = 0; i < H; i++) {
            for (unsigned int j = 0; j < W; j++) {
                size_t idx = i * W + j;
                if (idx >= map_.data.size()) continue;
                if (map_.data[idx] <= 0) continue;

                Vector2 cell_pos{
                    origin_x + j * resolution + resolution / 2.0f,
                    origin_y + i * resolution + resolution / 2.0f};

                Vector2 diff = cell_pos - pose_odom;
                float dist_sq = squared_euclidan_norm(diff);

                if (dist_sq < boid_vision_range_ * boid_vision_range_ &&
                    dist_sq < smallest_distance)
                {
                    smallest_distance = dist_sq;
                    closest_obstacle = cell_pos;
                }
            }
        }
        return closest_obstacle;
    }

    std::vector<Msg_Odom> get_graph_neighbours(
        const std::vector<Msg_Odom>& robot_odoms,
        unsigned int id)
    {
        if (id >= robot_odoms.size()) return {};

        std::vector<Msg_Odom> neighbours;
        neighbours.reserve(robot_odoms.size() - 1);

        for (size_t i = 0; i < robot_odoms.size(); ++i) {
            if (i != id) neighbours.push_back(robot_odoms[i]);
        }
        return neighbours;
    }

    void Subscription_cfID_PoseStamped_Callback(
        const Msg_PoseStamped::SharedPtr msg,
        unsigned int cfID)
    {
        if (cfID >= robot_odoms_.size()) return;

        robot_odoms_[cfID].pose.pose = msg->pose;

        Msg_Boid_Info odom_array_id;
        odom_array_id.odometries =
            get_graph_neighbours(robot_odoms_, cfID);

        odom_array_id.odometries.insert(
            odom_array_id.odometries.begin(),
            robot_odoms_[cfID]);

        pubs_boid_info_[cfID]->publish(odom_array_id);
    }

    void Subscription_cfID_LogDataGeneric_Callback(
        const Msg_LogDataGeneric::SharedPtr msg,
        unsigned int cfID)
    {
        if (cfID >= robot_odoms_.size()) return;

        if (msg->values.size() >= 3) {
            robot_odoms_[cfID].twist.twist.linear.x = msg->values[0];
            robot_odoms_[cfID].twist.twist.linear.y = msg->values[1];
            robot_odoms_[cfID].twist.twist.linear.z = msg->values[2];
        }
    }

    void Map_Callback(const Msg_Map::SharedPtr map)
    {
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
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

float normalize_angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}
