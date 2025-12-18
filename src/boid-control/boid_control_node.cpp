#include <algorithm>
#include <cassert>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include "defines.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>
#include "utils.h"
#include "vector2.hpp"

// ctrl+f sections:
// DEFINES, GLOBALS FOR TUNNING
// ROS2 NODE 
// MAIN 
// FUNCTION DEFINITIONS

// --------------------------------------------------------------
// DEFINES, GLOBALS FOR TUNNING

#define DEFAULT_RANGE_COHESION 1.0f
#define DEFAULT_RANGE_ALIGNMENT 1.0f
#define DEFAULT_RANGE_AVOIDANCE 1.0f

// --------------------------------------------------------------
// ROS2 NODE 

using std::placeholders::_1;
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
float normalize_angle(float angle);

struct Boid
{
    bool initialized = false;
    float last_rotation = 0.0f; 
    clock_t last_time = 0;
};

class Boid_Controller_Node : public rclcpp::Node
{
public:
    Boid_Controller_Node() : Node("boid_controller_node")
    {
        // get all available topics and extract highest <number> from topics with "robot_<number>/odom" 
        robot_id_ = this->declare_parameter<int>("robot_id", 0);

        // initialize all publishers, subscriptions and boids
        std::string robot_twist_topic = "/cf_" + std::to_string(int(robot_id_)) + "/cmd_vel";
        std::string robot_boid_info_topic = "/robot_" + std::to_string(int(robot_id_)) + "/boid_info";
        std::string robot_tuning_params_topic = "/tuning_params";
        std::string goal_odom_topic = "/goal_point";

        // init subs
        sub_boid_info = this->create_subscription<Msg_Boid_Info>(
            robot_boid_info_topic,
            10, 
            std::bind(&Boid_Controller_Node::Subscription_Boid_Info_Callback, this, _1)
        ); 
        
        sub_tuning_params = this->create_subscription<Msg_Tuning_Params>(
            robot_tuning_params_topic, 
            10, 
            std::bind(&Boid_Controller_Node::Subscription_Tuning_Params_Callback, this, _1)
        );

        sub_goal_odom = this->create_subscription<Msg_Point>(
            goal_odom_topic, 
            10, 
            std::bind(&Boid_Controller_Node::Subscription_Goal_Odom_Callback, this, _1)
        );

        // init pub
        pub_twist = this->create_publisher<Msg_Twist>(
                robot_twist_topic, 
                10
            ); 

        // init boid
        Boid boid;
        boid.initialized = false;
    }
private:
    // Project 1:
    void Subscription_Boid_Info_Callback(const Msg_Boid_Info::SharedPtr info);   // ALL BOID LOGIC HAPPENS HERE
    void Subscription_Tuning_Params_Callback(const Msg_Tuning_Params::SharedPtr params);
    void Subscription_Goal_Odom_Callback(const Msg_Point::SharedPtr point);
    Vector2 Calculate_Accel_Alignment(std::vector<Msg_Odom> odoms);
    Vector2 Calculate_Accel_Avoidence(std::vector<Msg_Odom> odoms);
    Vector2 Calculate_Accel_Cohesion(std::vector<Msg_Odom> odoms);
    Vector2 Calculate_Accel_Obstacle_Avoid(const Msg_Odom& self_odom, const Msg_Point& closest_obstacle);
    Vector2 Calculate_Accel_Goal(const Msg_Odom& self_odom, const Msg_Point& goal);
    // Project 2:
    // TODO: - Get_Adjacency_Matrix should read a line of the form "T: X" where X is the total number of boids to include in the consensus protocol (usually the same as the total number of boids presumably, but since only data_splitter gets to see the total number of boids and we could potentially say that some boids aren't part of the consensus protocol, these should be kept separate)
    //       - Store the adjacency matrix in a private variable
    //       - use Calculate_Vel_Centroid_Consensus to calculate consensus speeds and add it to the speeds at the end of Subscription_Boid_Info_Callback
    //       - in testing, use the separation rule but switch all other boid rules off
    //       - maybe implement using Goal on the Offset so that you can send the boids to a desired consensus-location without the boids' Go To Goal rule
    //               - (optional) implement separation within consensus protocol (recommended by the instructions in the first place), instead of using the boid separation rules
    std::vector<std::vector<bool>> Get_Adjancency_Matrix(std::string filepath, int num_boids);
    Vector2 Calculate_Vel_Centroid_Consensus(const std::vector<Msg_Odom> odoms);
    Vector2 Calculate_Vel_Centroid_Consensus(const std::vector<Msg_Odom> odoms, const std::vector<Vector2> offsets);

private:
    Boid boid;
    Publisher_Twist pub_twist;
    Subscription_Boid_Info sub_boid_info;
    Subscription_Tuning_Params sub_tuning_params;
    Subscription_Point sub_goal_odom;

    // parameter declarations 
    float robot_id_;
    // modifiable params
    bool params_init = false;
    float rotation_kp_;
    float cohesion_range_;
    float cohesion_factor_;
    float alignment_range_;
    float alignment_factor_;
    float avoidance_range_;
    float avoidance_factor_;
    float obstacle_avoid_range_;
    float obstacle_avoid_factor_;
    float goal_factor_;
    float max_speed_;

    Msg_Point goal_point;
};

// --------------------------------------------------------------
// MAIN 

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Boid_Controller_Node>());
    rclcpp::shutdown();
    return 0;
}

// --------------------------------------------------------------
// FUNCTION DEFINITIONS

// ALL BOITS LOGIC HAPPENS HERE
// TLDR: controls the boid based on boid_info message
// not TLDR: Calculates forces from all of the rules based on available information in Msg_Boid_Info
// return: nothing 
void Boid_Controller_Node::Subscription_Boid_Info_Callback(const Msg_Boid_Info::SharedPtr info)
{
    assert(info->odometries.size() && "info odom array should never be completely empty, odom of boid self is always the first element");
    
    // dont want to perform any calculations until params are set
    if (!params_init)
        return;

    // update information we have and which is required for later operations
    Msg_Odom self_odom = info->odometries[0];
    Msg_Point obstacle_point = info->closest_obstacle; 
    float z = self_odom.pose.pose.orientation.z;
    float w = self_odom.pose.pose.orientation.w;
    boid.last_rotation = atan2(2.0f * (w * z), w * w - z * z);

    // we need this to function and the garantee that at least single iteration has passed
    if (!boid.initialized)
    {
        boid.last_time = clock();
        boid.initialized = true;
        return;
    }

    // ---------------------------------------------------------
    // boids logic
    
    Vector2 accel_align = Calculate_Accel_Alignment(info->odometries); 
    Vector2 accel_avoid = Calculate_Accel_Avoidence(info->odometries);
    Vector2 accel_cohes = Calculate_Accel_Cohesion(info->odometries);
    Vector2 accel_obsta = Calculate_Accel_Obstacle_Avoid(self_odom, obstacle_point);
    Vector2 accel_goal  = Calculate_Accel_Goal(self_odom, goal_point);
    Vector2 accel_total = {};

    Vector2 vel_consensus  = Calculate_Vel_Centroid_Consensus(self_odom, goal_point);

    // combine forces forces
    {
        accel_total.x = 
            accel_align.x + 
            accel_avoid.x + 
            accel_cohes.x + 
            accel_obsta.x +
            accel_goal.x;

        accel_total.y = 
            accel_align.y + 
            accel_avoid.y +
            accel_cohes.y + 
            accel_obsta.y +
            accel_goal.y;
    }

    // create twist message based on total accel and current state well give to boid
    // and publish it
    {
        // delta time in seconds
        float delta_time = (double)(clock() - boid.last_time) / CLOCKS_PER_SEC;

        // calculate delta_velocities
        float delta_linear_x = sqrt(accel_total.x*accel_total.x + accel_total.y*accel_total.y); 
        // UPITNO za delta_linear_x jeli ok izracunati accel u namjenjenom za vremenski interval (t+1)-(t) 
        // a koristiti dt=(t)-(t-1)
        //
        // Dinov komentar: mislim da je ok jer je simulation refresh rate nadam se capped na necemu
        float yaw = getYawFromQuaternion(self_odom.pose.pose.orientation);
        
        float relative_angle = normalize_angle(atan2(accel_total.y, accel_total.x) - yaw);
        float delta_angular_z = p_controller_update(
                    atan2(accel_total.y, accel_total.x), 
                    boid.last_rotation,
                    this->rotation_kp_
                );

        // construct message
        Msg_Twist twist_msg;
      
        {
            twist_msg.linear.x = self_odom.twist.twist.linear.x + accel_total.x * delta_time + vel_consensus.x;
            twist_msg.linear.y = self_odom.twist.twist.linear.y + accel_total.y * delta_time + vel_consensus.y;
            twist_msg.angular.z = 0.0f;
            float norm = sqrt(twist_msg.linear.x*twist_msg.linear.x + twist_msg.linear.y*twist_msg.linear.y);
            if (norm > max_speed_)
            {
                twist_msg.linear.x /= norm;
                twist_msg.linear.y /= norm;
                twist_msg.linear.x *= max_speed_;
                twist_msg.linear.y *= max_speed_;
            }
        }
     
        twist_msg.linear.z = 0.0f;
        twist_msg.angular.x = 0.0f;
        twist_msg.angular.y = 0.0f;

        // Project 2
        //Vector2 vels = Calculate_Vel_Centroid_Consensus(odoms)
        //twist_msg.linear.x += vels.x;
        //twist_msg.linear.y += vels.y;

        pub_twist->publish(twist_msg);
    }

    // ---------------------------------------------------------
    // end of boids logic
    
    boid.last_time = clock();  // this is for delta_time to work
}

// TLDR: calculates alignment rule influence 
// not TLDR: Calculates acceleration based from alignment rule influence, with respect to odometries of self and other boids
// return: vector2 of acceleration 
Vector2 Boid_Controller_Node::Calculate_Accel_Alignment(std::vector<Msg_Odom> odoms)
{
    Vector2 force_alignment = {};
    uint alignment_neighbours_count = 0;

    Msg_Odom self_odom = odoms[0];

    Vector2 boid_self_pose = {
        (float)self_odom.pose.pose.position.x,
        (float)self_odom.pose.pose.position.y,
    };

    // start from 1 because 0 is boid self
    for (uint i = 1; i < odoms.size(); ++i)
    {
        Vector2 boid_i_pose = {
            (float)odoms[i].pose.pose.position.x,
            (float)odoms[i].pose.pose.position.y,
        };
        // distance between boid[id] and boid[i]
        Vector2 distance = {
            boid_i_pose.x - boid_self_pose.x,
            boid_i_pose.y - boid_self_pose.y
        };
    
        // is the boid[i] close enough to boid[id] take effect on it?
        if (squared_euclidan_norm(distance) <= (this->alignment_range_ * this->alignment_range_))
        {
            // linear component of linear+angular pair
            float boid_i_lin = odoms[i].twist.twist.linear.x;
            float boid_self_lin = self_odom.twist.twist.linear.x;

            // -calculating jaw based on quaternion data
            // -x and y are unneccessary because were only rotating around jaw so they are always 0
            // -full function if ever needed:
            // // float boid_i_rot = atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
            float z = (float)odoms[i].pose.pose.orientation.z;
            float w = (float)odoms[i].pose.pose.orientation.w;
            float boid_i_rot = atan2(2.0f * (w * z), w * w - z * z);
            z = (float)self_odom.pose.pose.orientation.z;
            w = (float)self_odom.pose.pose.orientation.w;
            float boid_self_rot = atan2(2.0f * (w * z), w * w - z * z);

            // -jaw = 0° is to the right of the screen
            // -positive rotation axis is right handed if Z goes outside the screen
            // -positive X is right, positive Y is up

            // -we only calculate linear_xy vectors based on magnitude and rotation
            // because angular velocity doesnt have affect on current_velocity_xy vector
            // -angular velocity is only needed for rotating the vessel which we will
            // calculate for actuation purposes, and not guidence, as it can be looked at as
            // acceleration thats affecting the magnitude vector in global frame 
            // (changes its value overtime in global frame, yes, twist.linear is in
            // local coordinate frame)
            Vector2 boid_i_vel_lin = {
                (float)cos(boid_i_rot)*boid_i_lin,
                (float)sin(boid_i_rot)*boid_i_lin,
            };
            Vector2 boid_id_vel_lin = {
                (float)cos(boid_self_rot)*boid_self_lin,
                (float)sin(boid_self_rot)*boid_self_lin,
            };

            force_alignment.x += boid_i_vel_lin.x - boid_id_vel_lin.x;
            force_alignment.y += boid_i_vel_lin.y - boid_id_vel_lin.y;

            ++alignment_neighbours_count;
        }
    }
    
    // scale force
    if (alignment_neighbours_count != 0)
    {
        force_alignment.x /= alignment_neighbours_count;
        force_alignment.y /= alignment_neighbours_count;
    }
    
    force_alignment.x *= alignment_factor_;
    force_alignment.y *= alignment_factor_;

    return force_alignment;
}

// TLDR: calculates aboidence rule influence 
// not TLDR: Calculates acceleration based from avoidence rule influence, with respect to odometries of self and other boids
// return: vector2 of acceleration 
Vector2 Boid_Controller_Node::Calculate_Accel_Avoidence(std::vector<Msg_Odom> odoms)
{
    Vector2 force_avoidence = {};

    Msg_Odom self_odom = odoms[0];

    Vector2 boid_self_pose = {
        (float)self_odom.pose.pose.position.x,
        (float)self_odom.pose.pose.position.y,
    };

    // start from 1 because 0 is boid self
    for (uint i = 1; i < odoms.size(); ++i)
    {
        Vector2 boid_i_pose = {
            (float)odoms[i].pose.pose.position.x,
            (float)odoms[i].pose.pose.position.y,
        };
        // distance between boid[id] and boid[i]
        Vector2 distance = {
            boid_self_pose.x - boid_i_pose.x,
            boid_self_pose.y - boid_i_pose.y
        };
    
        // is the boid[i] close enough to boid[id] take effect on it?
        if (squared_euclidan_norm(distance) <= (this->avoidance_range_ * this->avoidance_range_))
        {
            force_avoidence.x += distance.x / (squared_euclidan_norm(distance));
            force_avoidence.y += distance.y / (squared_euclidan_norm(distance));
        }
    }

    force_avoidence.x *= avoidance_factor_;
    force_avoidence.y *= avoidance_factor_;

    return force_avoidence;
}

// TLDR: calculates cohesion rule influence 
// not TLDR: Calculates acceleration based from cohesion rule influence, with respect to odometries of self and other boids
// return: vector2 of acceleration 
Vector2 Boid_Controller_Node::Calculate_Accel_Cohesion(std::vector<Msg_Odom> odoms)
{
    Vector2 force_cohesion = {};
    uint cohesion_neighbours_count = 0;
    
    Msg_Odom self_odom = odoms[0];

    Vector2 boid_self_pose = {
        (float)self_odom.pose.pose.position.x,
        (float)self_odom.pose.pose.position.y,
    };

    // start from 1 because 0 is boid self
    for (uint i = 1; i < odoms.size(); ++i)
    {
        Vector2 boid_i_pose = {
            0.0f,
            0.0f
        };
        // distance between boid[id] and boid[i]
        Vector2 distance = {
            boid_i_pose.x - boid_self_pose.x,
            boid_i_pose.y - boid_self_pose.y
        };
    
        // is within range?
        if (squared_euclidan_norm(distance) <= (this->cohesion_range_ * this->cohesion_range_))
        {
            force_cohesion.x += distance.x;
            force_cohesion.y += distance.y;
            ++cohesion_neighbours_count;
        }
    }

    // scale force
    if (cohesion_neighbours_count != 0)
    {
        force_cohesion.x /= cohesion_neighbours_count;
        force_cohesion.y /= cohesion_neighbours_count;
    }

    force_cohesion.x *= cohesion_factor_;
    force_cohesion.y *= cohesion_factor_;

    return force_cohesion;
}

// TLDR: calculates obstacle avoidence rule influence 
// not TLDR: Calculates acceleration based on closest obstacle point coordinates, with respect to odometry of self
// return: vector2 of acceleration 
Vector2 Boid_Controller_Node::Calculate_Accel_Obstacle_Avoid(const Msg_Odom& self_odom, const Msg_Point& closest_obstacle)
{
    Vector2 force_obstacle = {};

    Vector2 boid_self_pose = {
        (float)self_odom.pose.pose.position.x,
        (float)self_odom.pose.pose.position.y,
    };

    Vector2 closest_obstacle_pose = {
        (float)closest_obstacle.x,
        (float)closest_obstacle.y,
    };
   
    Vector2 distance = {
        boid_self_pose.x - closest_obstacle_pose.x,
        boid_self_pose.y - closest_obstacle_pose.y
    };
   
    // is within range?
    if (squared_euclidan_norm(distance) <= (this->obstacle_avoid_range_ * this->obstacle_avoid_range_))
    {
        force_obstacle.x += distance.x / (squared_euclidan_norm(distance));
        force_obstacle.y += distance.y / (squared_euclidan_norm(distance));
    }

    force_obstacle.x *= obstacle_avoid_factor_;
    force_obstacle.y *= obstacle_avoid_factor_;
    return force_obstacle;
}

// TLDR: updates tuning params 
// not TLDR: Based on subscription of topic thats designed to update tuning params, updates tuning params 
// return: nothing
void Boid_Controller_Node::Subscription_Tuning_Params_Callback(const Msg_Tuning_Params::SharedPtr params)
{
    params_init = true; 
    
    rotation_kp_ = params->rotation_kp;
    cohesion_range_ = params->cohesion_range;
    cohesion_factor_ = params->cohesion_factor; 
    alignment_range_ = params->alignment_range; 
    alignment_factor_ = params->alignment_factor; 
    avoidance_range_ = params->avoidance_range; 
    avoidance_factor_ = params->avoidance_factor;
    obstacle_avoid_range_ = params->obstacle_avoid_range; 
    obstacle_avoid_factor_ = params->obstacle_avoid_factor; 
    goal_factor_ = params->goal_factor; 
    max_speed_ = params->max_speed; 
}

// TLDR: updates tuning params 
// not TLDR: Based on subscription of topic thats designed to update tuning params, updates tuning params 
// return: nothing
void Boid_Controller_Node::Subscription_Goal_Odom_Callback(const Msg_Point::SharedPtr point)
{
    goal_point = *point;
}

// TLDR: calculates goal following rule influence 
// not TLDR: Calculates acceleration based from goal following rule influence, with respect to odometries of self and goal 
// return: vector2 of acceleration 
Vector2 Boid_Controller_Node::Calculate_Accel_Goal(const Msg_Odom& self_odom, const Msg_Point& goal_point)
{
    Vector2 force_goal;

    Vector2 self_pose = {
        (float)self_odom.pose.pose.position.x,
        (float)self_odom.pose.pose.position.y,
    };

    Vector2 goal_pose = {
        (float)goal_point.x,
        (float)goal_point.y,
    };

    Vector2 difference = goal_pose - self_pose;
    force_goal = difference / difference.length_squared();
    force_goal *= goal_factor_; 
    
    return force_goal;
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

/// @brief Computes velocities for current agent to move towards the centroid of the agents it has the information of
/// @param odoms Array of all odometry messages that the robot can see.
/// @return 
Vector2 Boid_Controller_Node::Calculate_Vel_Centroid_Consensus(const std::vector<Msg_Odom> odoms)
{
    Vector2 directed_total = {
        (float)0,
        (float)0,
    };

    for (int i = 0; i < odoms.size(); i++) {
        directed_total.x += (float)odoms[i].pose.pose.position.x - (float)odoms[robot_id_-1].pose.pose.position.x;
        directed_total.y += (float)odoms[i].pose.pose.position.y - (float)odoms[robot_id_-1].pose.pose.position.y;
    }

    return directed_total * (float)((double)(clock() - boid.last_time) / CLOCKS_PER_SEC);
}

/// @brief Computes velocities for current agent to move towards the centroid of the agents it has the information of
/// @param odoms Array of all odometry messages that the robot can see.
/// @param offsets Array of all offsets that the robot should use in the formation.
/// @return 
Vector2 Boid_Controller_Node::Calculate_Vel_Centroid_Consensus(const std::vector<Msg_Odom> odoms, const std::vector<Vector2> offsets)
{
    Vector2 directed_total = {
        (float)0,
        (float)0,
    };

    for (int i = 0; i < odoms.size(); i++) {
        directed_total.x += (float)odoms[i].pose.pose.position.x - (float)odoms[robot_id_-1].pose.pose.position.x;
        directed_total.y += (float)odoms[i].pose.pose.position.y - (float)odoms[robot_id_-1].pose.pose.position.y;

        directed_total -= offsets[i] - offsets[robot_id_-1];
    }

    return directed_total * (float)((double)(clock() - boid.last_time) / CLOCKS_PER_SEC);
}

std::vector<std::vector<bool>> Get_Adjancency_Matrix(std::string filepath, int num_boids)
    {
        typedef std::vector<std::vector<bool>> bool_matrix;
        bool_matrix mat;
        
        std::ifstream in(filepath);

        if (in.bad())
        {
            RCUTILS_LOG_FATAL("INPUT CONFIG FILE %s IS BAD", filepath.c_str());
            return mat;
        }

        mat.reserve(num_boids);
        for (auto& cell : mat)
            cell.reserve(num_boids);

        for (size_t y = 0; y < num_boids; y++)
            for (size_t x = 0; x < num_boids; x++)
                mat[y][x] = 0;

        if (in.is_open())
        {
            std::string line = "";
            while (!in.eof()) 
            {
                std::getline(in, line);
                if (line.size() && !(line[0] >= '0' && line[0] <= '9'/* || line[0] == 'T' */)) // T: <num> compatibility for total number of bots to include in consensus (consensus protocol needs to be calculated in boid_control_node, so the building of the adjacency matrix needs to use boid_num but since that's not present in boid_control we have to get it manually)
                    continue;
                
                std::string temp;

                int left = -1;
                std::vector<int> right; 
                // parsing stages
                // line "left:right"
                // stage 1, find "left:"
                // stage 2, find ":right"
                {
                    size_t i = 0;

                    // stage 1
                    {
                        temp = "";
                        while (left == -1 && i < line.size())
                        {
                            char c = line[i++];
                            if (c == ':')
                                left = std::stoi(temp);
                            else
                                temp += c;
                        }
                    }

                    // stage 2
                    {
                        temp = "";
                        while (i < line.size()) 
                        {
                            char c = line[i++];
                            if (c == ' ')
                            {
                                right.push_back(std::stoi(temp));
                                temp = "";
                            }
                            else
                                temp += c;
                        }

                        if (temp != "")
                        {
                            right.push_back(std::stoi(temp));
                            temp = "";
                        }
                    }
                } 
            
                for (const int r : right)
                {
                    mat[left][r] = '1';
                    mat[r][left] = '1';
                }
            }
        }

        in.close();

        return mat;
    }
