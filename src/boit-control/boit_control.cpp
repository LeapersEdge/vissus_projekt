#include <cassert>
#include <cmath>
#include <csignal>
#include <cstdio>
#include "defines.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <string>
#include <vector>
#include "utils.h"

// --------------------------------------------------------------
// DEFINES, GLOBALS FOR TUNNING

#define DEFAULT_RANGE_COHESION 1.0f
#define DEFAULT_RANGE_ALIGNMENT 1.0f
#define DEFAULT_RANGE_AVOIDANCE 1.0f

// --------------------------------------------------------------
// ROS2 NODE 

using std::placeholders::_1;

class Boit_Controller_Node : public rclcpp::Node
{
public:
    Boit_Controller_Node() : Node("boit_controller_node")
    {
        // get all available topics and extract highest <number> from topics with "robot_<number>/odom" 
        rotation_kp_        = this->declare_parameter<float>("rotation_kp", 0.0f);
        containment_force_  = this->declare_parameter<float>("containment_force", 0.0f);
        cohesion_range_     = this->declare_parameter<float>("cohesion_range", DEFAULT_RANGE_COHESION);
        cohesion_factor_    = this->declare_parameter<float>("cohesion_factor", 0.0f);
        alignment_range_    = this->declare_parameter<float>("alignment_range", DEFAULT_RANGE_ALIGNMENT);
        alignment_factor_   = this->declare_parameter<float>("alignment_factor", 0.0f);
        avoidance_range_    = this->declare_parameter<float>("avoidance_range", DEFAULT_RANGE_AVOIDANCE);
        avoidance_factor_   = this->declare_parameter<float>("avoidance_factor", 0.0f);

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
      
        // initialize all publishers, subscriptions and boits
        for (uint i = 0; i < highest_robot_id; ++i)
        {
            std::string robot_twist_topic = "/robot_" + std::to_string(i) + "/cmd_vel";
            std::string robot_boit_info_topic = "/robot_" + std::to_string(i) + "/boit_info";
        
            Subscription_Boit_Info sub = this->create_subscription<Msg_Boit_Info>(
                    robot_boit_info_topic, 
                    10, 
                    [this, i](const Msg_Boit_Info::SharedPtr msg) {
                        this->Subscription_Boit_Info_Callback(msg, i);
                    }
                ); 
            subs.push_back(sub);
            
            // init pub
            Publisher_Twist pub = this->create_publisher<Msg_Twist>(
                    robot_twist_topic, 
                    10
                ); 
            pubs.push_back(pub);

            // init boit
            Boit boit;
            boit.initialized = false;
            boits.push_back(boit);
        }
    }
private:
    void Subscription_Boit_Info_Callback(const Msg_Boit_Info::SharedPtr info, uint id);   // ALL BOITS LOGIC HAPPENS HERE
    Vector2 Calculate_Accel_Alignment(std::vector<Msg_Odom> odoms);
    Vector2 Calculate_Accel_Avoidence(std::vector<Msg_Odom> odoms);
    Vector2 Calculate_Accel_Cohesion(std::vector<Msg_Odom> odoms);

private:
    std::vector<Boit> boits;
    std::vector<Publisher_Twist> pubs;
    std::vector<Subscription_Boit_Info> subs;

    //parameters declarations 
    float rotation_kp_;
    float containment_force_;
    float cohesion_range_;
    float cohesion_factor_;
    float alignment_range_;
    float alignment_factor_;
    float avoidance_range_;
    float avoidance_factor_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Boit_Controller_Node>());
    rclcpp::shutdown();
    return 0;
}

// ALL BOITS LOGIC HAPPENS HERE
void Boit_Controller_Node::Subscription_Boit_Info_Callback(const Msg_Boit_Info::SharedPtr info, uint id)
{
    assert(id < boits.size() && "boit id should not surpass the size of boits container");
    assert(info->odometries.size() && "info odom array should never be completely empty, odom of boit self is always the first element");
    
    // update information we have
    Msg_Odom self_odom = info->odometries[0];
    float z = self_odom.pose.pose.orientation.z;
    float w = self_odom.pose.pose.orientation.w;
    boits[id].last_rotation = atan2(2.0f * (w * z), w * w - z * z);

    if (!boits[id].initialized)
    {
        boits[id].last_time = clock();
        boits[id].initialized = true;
        return;
    }

    // ---------------------------------------------------------
    // boits logic
    
    Vector2 accel_align = Calculate_Accel_Alignment(info->odometries); 
    Vector2 accel_avoid = Calculate_Accel_Avoidence(info->odometries);
    Vector2 accel_cohes = Calculate_Accel_Cohesion(info->odometries);
    Vector2 accel_total = {};

    // combine forces forces
    {
        // converts within 20% to the wall
        // working area ~~ X€<-4.9,4.9>, Y€<-4.9,4.9>
        Vector2 accel_containment = {};
        {
            Vector2 boit_self_pose = {
                (float)self_odom.pose.pose.position.x,
                (float)self_odom.pose.pose.position.y,
            };

            if (boit_self_pose.x > 4.9*0.8)
                accel_containment.x = -this->containment_force_;
            if (boit_self_pose.x < -4.9*0.8)
                accel_containment.x = this->containment_force_;
            if (boit_self_pose.y > 4.9*0.8)
                accel_containment.y = -this->containment_force_;
            if (boit_self_pose.y < -4.9*0.8)
                accel_containment.y = this->containment_force_;
        } 
        
        accel_total.x = 
            accel_align.x + 
            accel_avoid.x + 
            accel_cohes.x + 
            accel_containment.x;

        accel_total.y = 
            accel_align.y + 
            accel_avoid.y + 
            accel_cohes.y + 
            accel_containment.y;

    }

    // create twist message based on total accel and current state well give to boit
    // and publish it
    {
        // delta time in seconds
        float delta_time = (double)(clock() - boits[id].last_time) / CLOCKS_PER_SEC;

        // calculate delta_velocities
        float delta_linear_x = sqrt(accel_total.x*accel_total.x + accel_total.y*accel_total.y) * delta_time; 
        // UPITNO za delta_linear_x jeli ok izracunati accel u namjenjenom za vremenski interval (t+1)-(t) 
        // a koristiti dt=(t)-(t-1)
        //
        // Dinov komentar: mislim da je ok jer je simulation refresh rate nadam se capped na necemu
        float delta_angular_z = p_controller_update(
                    atan2(accel_total.y, accel_total.x), 
                    boits[id].last_rotation, 
                    this->rotation_kp_
                );


        // construct message
        Msg_Twist twist_msg;
        twist_msg.linear.x = self_odom.twist.twist.linear.x + delta_linear_x;
        twist_msg.linear.y = 0.0f;
        twist_msg.linear.z = 0.0f;
        twist_msg.angular.x = 0.0f;
        twist_msg.angular.y = 0.0f;
        twist_msg.angular.z = self_odom.twist.twist.angular.z + delta_angular_z;

        pubs[id]->publish(twist_msg);
    }

    // ---------------------------------------------------------
    // end of boits logic
    
    boits[id].last_time = clock();  // this is for delta_time to work
}

Vector2 Boit_Controller_Node::Calculate_Accel_Alignment(std::vector<Msg_Odom> odoms)
{
    Vector2 force_alignment = {};
    uint alignment_neighbours_count = 0;

    Msg_Odom self_odom = odoms[0];

    Vector2 boit_self_pose = {
        (float)self_odom.pose.pose.position.x,
        (float)self_odom.pose.pose.position.y,
    };

    // start from 1 because 0 is boit self
    for (uint i = 1; i < odoms.size(); ++i)
    {
        Vector2 boit_i_pose = {
            (float)odoms[i].pose.pose.position.x,
            (float)odoms[i].pose.pose.position.y,
        };
        // distance between boit[id] and boit[i] 
        Vector2 distance = {
            boit_i_pose.x - boit_self_pose.x,
            boit_i_pose.y - boit_self_pose.y
        };
    
        // is the boit[i] close enough to boit[id] take effect on it?
        if (squared_euclidan_norm(distance) <= (this->alignment_range_ * this->alignment_range_))
        {
            // linear component of linear+angular pair
            float boit_i_lin = odoms[i].twist.twist.linear.x;
            float boit_self_lin = self_odom.twist.twist.linear.x;

            // -calculating jaw based on quaternion data
            // -x and y are unneccessary because were only rotating around jaw so they are always 0
            // -full function if ever needed:
            // // float boit_i_rot = atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
            float z = (float)odoms[i].pose.pose.orientation.z;
            float w = (float)odoms[i].pose.pose.orientation.w;
            float boit_i_rot = atan2(2.0f * (w * z), w * w - z * z);
            z = (float)self_odom.pose.pose.orientation.z;
            w = (float)self_odom.pose.pose.orientation.w;
            float boit_self_rot = atan2(2.0f * (w * z), w * w - z * z);

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
            Vector2 boit_i_vel_lin = {
                (float)cos(boit_i_rot)*boit_i_lin,
                (float)sin(boit_i_rot)*boit_i_lin,
            };
            Vector2 boit_id_vel_lin = {
                (float)cos(boit_self_rot)*boit_self_lin,
                (float)sin(boit_self_rot)*boit_self_lin,
            };

            force_alignment.x += boit_i_vel_lin.x - boit_id_vel_lin.x;
            force_alignment.y += boit_i_vel_lin.y - boit_id_vel_lin.y;

            ++alignment_neighbours_count;
        }
    }
    
    if (alignment_neighbours_count != 0)
    {
        force_alignment.x /= alignment_neighbours_count;
        force_alignment.y /= alignment_neighbours_count;
    }
    
    force_alignment.x *= alignment_factor_;
    force_alignment.y *= alignment_factor_;

    return force_alignment;
}

Vector2 Boit_Controller_Node::Calculate_Accel_Avoidence(std::vector<Msg_Odom> odoms)
{
    Vector2 force_avoidence = {};

    Msg_Odom self_odom = odoms[0];

    Vector2 boit_self_pose = {
        (float)self_odom.pose.pose.position.x,
        (float)self_odom.pose.pose.position.y,
    };

    // start from 1 because 0 is boit self
    for (uint i = 1; i < odoms.size(); ++i)
    {
        Vector2 boit_i_pose = {
            (float)odoms[i].pose.pose.position.x,
            (float)odoms[i].pose.pose.position.y,
        };
        // distance between boit[id] and boit[i] 
        Vector2 distance = {
            boit_i_pose.x - boit_self_pose.x,
            boit_i_pose.y - boit_self_pose.y
        };
    
        // is the boit[i] close enough to boit[id] take effect on it?
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

Vector2 Boit_Controller_Node::Calculate_Accel_Cohesion(std::vector<Msg_Odom> odoms)
{
    Vector2 force_cohesion = {};
    uint cohesion_neighbours_count = 0;
    
    Msg_Odom self_odom = odoms[0];

    Vector2 boit_self_pose = {
        (float)self_odom.pose.pose.position.x,
        (float)self_odom.pose.pose.position.y,
    };

    // start from 1 because 0 is boit self
    for (uint i = 1; i < odoms.size(); ++i)
    {
        Vector2 boit_i_pose = {
            (float)odoms[i].pose.pose.position.x,
            (float)odoms[i].pose.pose.position.y,
        };
        // distance between boit[id] and boit[i] 
        Vector2 distance = {
            boit_i_pose.x - boit_self_pose.x,
            boit_i_pose.y - boit_self_pose.y
        };
    
        if (squared_euclidan_norm(distance) <= (this->cohesion_range_ * this->cohesion_range_))
        {
            force_cohesion.x += distance.x;
            force_cohesion.y += distance.y;
            ++cohesion_neighbours_count;
        }
    }

    if (cohesion_neighbours_count != 0)
    {
        force_cohesion.x /= cohesion_neighbours_count;
        force_cohesion.y /= cohesion_neighbours_count;
    }

    force_cohesion.x *= cohesion_factor_;
    force_cohesion.y *= cohesion_factor_;

    return force_cohesion;
}

