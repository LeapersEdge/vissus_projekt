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
#include "geometry_msgs/msg/pose_array.hpp"
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
#include "std_msgs/msg/int32_multi_array.hpp"


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

void printBoolMatrix(const std::vector<std::vector<bool>> &matrix)
{
    std::ostringstream oss;
    oss << "[\n";
    for (const auto &row : matrix)
    {
        oss << "  [ ";
        for (bool val : row)
        {
            oss << (val ? "true" : "false") << " ";
        }
        oss << "]\n";
    }
    oss << "]";
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n%s", oss.str().c_str());
}


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
	mode_ = this->declare_parameter<std::string>("mode", "market");
	//sep_range_ = this->declare_parameter<float>("separation_range", 0);
	//sep_factor_ = this->declare_parameter<float>("separantion_factor", 0);

	    std::string filepath = "/root/ros2_ws/src/vissus_projekt/launch/topology";
	    int num_boids = 4;
        //adjacency_mat_ = Boid_Controller_Node::Get_Adjancency_Matrix(filepath, 4);
        //printBoolMatrix(adjacency_mat_);
        adjacency_mat_ = {
            {false, true,  true,  true},  // Node 1 connected to 2, 3, 4
            {true,  false, true,  true},  // Node 2 connected to 1, 3, 4
            {true,  true,  false, true},  // Node 3 connected to 1, 2, 4
            {true,  true,  true,  false}  // Node 4 connected to 1, 2, 3
        };

	
	
        // initialize all publishers, subscriptions and boids
        std::string robot_twist_topic = "/cf_" + std::to_string(int(robot_id_)) + "/cmd_vel";
        std::string robot_boid_info_topic = "/robot_" + std::to_string(int(robot_id_)) + "/boid_info";
        std::string robot_tuning_params_topic = "/tuning_params";
        std::string goal_odom_topic = "/robot_goals";
    	std::string formation_topic = "/formation";
	std::string adjacency_topic = "/adjacency_matrix";
	
        // init subs
	sub_adjacency = this->create_subscription<Msg_Adjacency>(
            adjacency_topic,
            10, 
            std::bind(&Boid_Controller_Node::Subscription_Adjacency_Callback, this, _1)
        ); 
        
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

	    sub_formation = this->create_subscription<Msg_Formation>(
            formation_topic, 
            10, 
            std::bind(&Boid_Controller_Node::Subscription_Formation_Callback, this, _1)
        );
	
	    sub_goal = this->create_subscription<Msg_PoseArray>(
            goal_odom_topic, 
            10, 
            std::bind(&Boid_Controller_Node::Subscription_Goal_Callback, this, _1)
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
    void Subscription_Boid_Info_Callback(const Msg_Boid_Info::SharedPtr info);   // ALL BOID LOGIC HAPPENS HERE
    void Subscription_Tuning_Params_Callback(const Msg_Tuning_Params::SharedPtr params);
    void Subscription_Goal_Callback(const Msg_PoseArray::SharedPtr points);
    void Subscription_Formation_Callback(const Msg_Formation::SharedPtr points);
    void Subscription_Adjacency_Callback(const Msg_Adjacency::SharedPtr msg);

    Vector2 Calculate_Accel_Goal(const Msg_Odom& self_odom, const Msg_Point& goal);
    // Project 2:
    // TODO: - Get_Adjacency_Matrix should read a line of the form "T: X" where X is the total number of boids to include in the consensus protocol (usually the same as the total number of boids presumably, but since only data_splitter gets to see the total number of boids and we could potentially say that some boids aren't part of the consensus protocol, these should be kept separate)
    //       - Store the adjacency matrix in a private variable
    //       - use Calculate_Vel_Centroid_Consensus to calculate consensus speeds and add it to the speeds at the end of Subscription_Boid_Info_Callback
    //       - in testing, use the separation rule but switch all other boid rules off
    //       - maybe implement using Goal on the Offset so that you can send the boids to a desired consensus-location without the boids' Go To Goal rule
    //               - (optional) implement separation within consensus protocol (recommended by the instructions in the first place), instead of using the boid separation rules
    Bool_mat Get_Adjancency_Matrix(std::string filepath, int num_boids);    
    Vector2 Calculate_Vel_Centroid_Consensus(const std::vector<Msg_Odom> odoms);
    Vector2 Calculate_Vel_Centroid_Consensus(const std::vector<Msg_Odom> odoms, const std::vector<Vector2> offsets);
    Vector2 Calculate_Avoidance(const std::vector<Msg_Odom> odoms);     
    bool Connected(Bool_mat adjacency_mat, int robot_id);
    int sign(float x);

private:
    Boid boid;
    Publisher_Twist pub_twist;
    Subscription_Boid_Info sub_boid_info;
    Subscription_Tuning_Params sub_tuning_params;
    Subscription_PoseArray sub_goal;
    Subscription_Formation sub_formation;
    Subscription_Adjacency sub_adjacency;
    

    Bool_mat adjacency_mat_;

    // parameter declarations 
    int robot_id_;
    // modifiable params
    std::string mode_;
    bool params_init = false;
    bool formation_init = false;
    bool adjacency_init = false;
    bool robot_goals_init = false;
    bool first_take_off = true;
    float rotation_kp_;
    float census_factor_; 
    float avoidance_range_;
    float avoidance_factor_;       
    float goal_factor_;
    float max_speed_;
    Msg_Point goal_point_;
  
    std::vector<Vector2> offsets_;   
};

// --------------------------------------------------------------
// MAIN 

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);    rclcpp::spin(std::make_shared<Boid_Controller_Node>());
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
    
    //std::vector<Vector2> offsets_= {Vector2{0, 0}, Vector2{0.5, 0.5}, Vector2{1.0, 1.0}, Vector2{1.5, 1.5}};
    //Vector2 vel_consensus  = Calculate_Vel_Centroid_Consensus(info->odometries, offsets_);
    //std::vector<Vector2> offsets_= {Vector2{0, 0}, Vector2{1.0, 1.0}, Vector2{2.0, 2.0}, Vector2{3.0, 3.0}};
    Vector2 vel_consensus;


    // RCUTILS_LOG_INFO("mode is %s", mode_.c_str());
    // RCUTILS_LOG_INFO("adjacency_init is %d", adjacency_init);
    // RCUTILS_LOG_INFO("robot goals init is %d", robot_goals_init);
    printBoolMatrix(adjacency_mat_);
    if (mode_ == "rendevous")
    {
	vel_consensus = Calculate_Vel_Centroid_Consensus(info->odometries);
    }
    else if (mode_ == "formation")
    {
	if (!formation_init) return;	    						
	vel_consensus = Calculate_Vel_Centroid_Consensus(info->odometries, offsets_);	    
    }
    else if (mode_ == "market"){
	if (!adjacency_init || !robot_goals_init) return;
        vel_consensus = Calculate_Vel_Centroid_Consensus(info->odometries, offsets_);	    
    }	

    
    else
    {
	RCUTILS_LOG_ERROR("Mode must be either market, formation or rendevous, not %s", mode_.c_str());
	return;
    }
    // RCUTILS_LOG_INFO("Vel consenus goals init is (%f, %f)", vel_consensus.x, vel_consensus.y);
    
    vel_consensus.x = std::clamp(vel_consensus.x, -max_speed_, max_speed_);
    vel_consensus.y = std::clamp(vel_consensus.y, -max_speed_, max_speed_);

    Vector2 separation_consensus = Calculate_Vel_Centroid_Consensus(info->odometries);

    vel_consensus += separation_consensus;

    // create twist message based on total accel and current state well give to boid
    // and publish it
    {
        // delta time in seconds        
       
        // construct message
        Msg_Twist twist_msg;
	
	bool connected = Connected(adjacency_mat_, robot_id_);


    connected = true;
	if (first_take_off && connected)	    
	    {
		twist_msg.linear.z = 0.1f;
		first_take_off = false;
	    }

	else if (!connected)
	    {
		twist_msg.linear.z = -1.0f;
		first_take_off = true;
	    }
        twist_msg.angular.x = 0.0f;
        twist_msg.angular.y = 0.0f;

        // Project 2
        //Vector2 vels = Calculate_Vel_Centroid_Consensus(odoms)
	//RCUTILS_LOG_INFO("Vel consensus x:%f", vel_consensus.x);
	RCUTILS_LOG_INFO("Vel consensus connected: :%d", connected); 
	//RCUTILS_LOG_INFO("Vel consensus y:%f", vel_consensus.y);
        twist_msg.linear.x = vel_consensus.x;
        twist_msg.linear.y = vel_consensus.y;

        pub_twist->publish(twist_msg);
    }

    // ---------------------------------------------------------
    // end of boids logic
    
    boid.last_time = clock();  // this is for delta_time to work
}

// TLDR: updates tuning params 
// not TLDR: Based on subscription of topic thats designed to update tuning params, updates tuning params 
// return: nothing
void Boid_Controller_Node::Subscription_Tuning_Params_Callback(const Msg_Tuning_Params::SharedPtr params)
{
    params_init = true; 
    
    rotation_kp_ = params->rotation_kp;     
    avoidance_range_ = params->avoidance_range; 
    avoidance_factor_ = params->avoidance_factor;
    census_factor_ = params->census_factor;
    goal_factor_ = params->goal_factor; 
    max_speed_ = params->max_speed; 
}

// TLDR: updates tuning params 
// not TLDR: Based on subscription of topic thats designed to update tuning params, updates tuning params 
// return: nothing
void Boid_Controller_Node::Subscription_Goal_Callback(const Msg_PoseArray::SharedPtr poses)
{
    robot_goals_init = true;
    
    goal_point_ = poses->poses[robot_id_ - 1].position;

    offsets_.resize(poses->poses.size());

    for (size_t i = 0; i < poses->poses.size(); i++) {
	offsets_[i].x = poses->poses[i].position.x;
	offsets_[i].y = poses->poses[i].position.y;
    }

}

// void Boid_Controller_Node::Subscription_Formation_Callback(const Msg_Formation::SharedPtr points)
// {   
//   for (auto point : points->formation_points){
//     Vector2 vpoint;
//     vpoint.x = point.x;
//     vpoint.y = point.y;
//     offsets_.push_back(vpoint);        
//   }
// }

void Boid_Controller_Node::Subscription_Formation_Callback(const Msg_Formation::SharedPtr points)
{   
    offsets_ = { 
        Vector2{points->cf1.x, points->cf1.y},
        Vector2{points->cf2.x, points->cf2.y},
        Vector2{points->cf3.x, points->cf3.y},
        Vector2{points->cf4.x, points->cf4.y}
    };
}


void Boid_Controller_Node::Subscription_Adjacency_Callback(
    const Msg_Adjacency::SharedPtr msg)
{
    if (msg->layout.dim.size() != 2) {
        RCLCPP_ERROR(this->get_logger(),
                     "Adjacency matrix must be 2D MultiArray");
        return;
    }

    size_t rows = msg->layout.dim[0].size;
    size_t cols = msg->layout.dim[1].size;

    if (msg->data.size() != rows * cols) {
        RCLCPP_ERROR(this->get_logger(),
                     "Adjacency data size mismatch");
        return;
    }

    adjacency_mat_.assign(rows, std::vector<bool>(cols));

    for (size_t r = 0; r < rows; ++r) {
        for (size_t c = 0; c < cols; ++c) {
            size_t idx = r * cols + c;
            adjacency_mat_[r][c] = (msg->data[idx] != 0);
        }
    }

    adjacency_init = true;
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
    float dist2 = difference.length_squared();
    if (dist2 < 1e-6f)
	return Vector2{0, 0};

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
        if (adjacency_mat_[robot_id_-1][i]) {
            directed_total.x += (float)odoms[i].pose.pose.position.x - (float)odoms[robot_id_-1].pose.pose.position.x;
            directed_total.y += (float)odoms[i].pose.pose.position.y - (float)odoms[robot_id_-1].pose.pose.position.y;


	    Vector2 delta_pos = { // robot_i.pos - robot_cur.pos
	      (float)odoms[i].pose.pose.position.x - (float)odoms[robot_id_-1].pose.pose.position.x,
	      (float)odoms[i].pose.pose.position.y - (float)odoms[robot_id_-1].pose.pose.position.y,
	    };
	    float delta_dist = delta_pos.length();
	    
	    //directed_total.x += (float)odoms[i].pose.pose.position.x - (float)odoms[robot_id_-1].pose.pose.position.x;
	    //directed_total.y += (float)odoms[i].pose.pose.position.y - (float)odoms[robot_id_-1].pose.pose.position.y;
	    
	    directed_total += delta_pos;
	    
	    if (delta_dist < avoidance_range_) {
	      //Vector2 separation = delta_pos / delta_pos.length_squared()
	      Vector2 separation;
	      //separation.x = sign(delta_pos.x) / delta_pos.length_squared(); // * (avoidance_range_ - std::sqrt(squared_euclidan_norm(delta_pos)));
	      //separation.y = sign(delta_pos.y) / delta_pos.length_squared(); // * (avoidance_range_ - std::sqrt(squared_euclidan_norm(delta_pos)));
	      
	      separation.x = (1. - delta_dist / avoidance_range_) * delta_pos.x / delta_pos.length_squared(); // linear repulsion 
	      separation.y = (1. - delta_dist / avoidance_range_) * delta_pos.y / delta_pos.length_squared(); //
	      
	      separation *= avoidance_factor_; // maybe slow it down more smoothly? this is using squared distance...
	      separation /= delta_dist / avoidance_range_; // affect it by where it is proportional to the range at which separation starts
        
	      directed_total += separation;
	      
	    }
        } 
     }
	
    //  return directed_total * (float)((double)(clock() - boid.last_time) / CLOCKS_PER_SEC);
    return directed_total * 0.001f;
}

/// @brief Computes velocities for current agent to move towards the centroid of the agents it has the information of
/// @param odoms Array of all odometry messages that the robot can see.
/// @param offsets Array of all offsets that the robot should use in the formation.
/// @return 
Vector2 Boid_Controller_Node::Calculate_Vel_Centroid_Consensus(const std::vector<Msg_Odom> odoms, const std::vector<Vector2> offsets)
{
  //float sep_range = 1.0.f; // replace with actual separation rule range? this->avoidance_range_?
  //float sep_factor = 0.001f; // replace with actual separation rule factor?

    Vector2 directed_total = {
        (float)0,
        (float)0,
    };


    for (int i = 0; i < odoms.size(); i++) {
        if (i == robot_id_-1 || !adjacency_mat_[robot_id_-1][i]) continue;	        
	
        Vector2 delta_pos = { // robot_i.pos - robot_cur.pos
	  (float)odoms[i].pose.pose.position.x - (float)odoms[robot_id_-1].pose.pose.position.x,
	  (float)odoms[i].pose.pose.position.y - (float)odoms[robot_id_-1].pose.pose.position.y,
        };

	float delta_dist = delta_pos.length();

        //directed_total.x += (float)odoms[i].pose.pose.position.x - (float)odoms[robot_id_-1].pose.pose.position.x;
        //directed_total.y += (float)odoms[i].pose.pose.position.y - (float)odoms[robot_id_-1].pose.pose.position.y;

        directed_total += delta_pos - (offsets[i] - offsets[robot_id_-1]);
    }

    directed_total *= census_factor_ ;
    return directed_total;
}

Vector2 Boid_Controller_Node::Calculate_Avoidance(const std::vector<Msg_Odom> odoms)
{    
    Vector2 directed_total = {
        (float)0,
        (float)0,
    };


    for (int i = 0; i < odoms.size(); i++) {
        if (i == robot_id_-1 || !adjacency_mat_[robot_id_-1][i]) continue;
        
	
        Vector2 delta_pos = { // robot_i.pos - robot_cur.pos
	  (float)odoms[i].pose.pose.position.x - (float)odoms[robot_id_-1].pose.pose.position.x,
	  (float)odoms[i].pose.pose.position.y - (float)odoms[robot_id_-1].pose.pose.position.y,
        };

	float delta_dist = delta_pos.length();


        if (delta_dist < avoidance_range_) {
            Vector2 separation;
	    
	    separation.x = (1. - delta_dist / avoidance_range_) * delta_pos.x / delta_pos.length_squared(); // linear repulsion 
            separation.y = (1. - delta_dist / avoidance_range_) * delta_pos.y / delta_pos.length_squared(); //

            separation *= avoidance_factor_; 
            separation /= delta_dist / avoidance_range_; // affect it by where it is proportional to the range at which separation starts
        
            directed_total = separation;
        }
    }

    directed_total *= census_factor_ * avoidance_factor_;
    return directed_total;
}

int Boid_Controller_Node::sign(float x) {
    return (x < 0.0f) ? -1 : 1;
}

bool Boid_Controller_Node::Connected(Bool_mat adjacency_mat, int robot_id)
{
    bool has_neighbor = false;

    if (adjacency_init && robot_id - 1 < adjacency_mat.size())
	{
	    for (bool connected : adjacency_mat[robot_id - 1])
		{
		    if (connected) {
			has_neighbor = true;
			break;
		    }
		}
	}
}


Bool_mat Boid_Controller_Node::Get_Adjancency_Matrix(std::string filepath, int num_boids)
{
    Bool_mat mat;

    std::ifstream in(filepath);

    if (in.bad())
    {
        RCUTILS_LOG_FATAL("INPUT CONFIG FILE %s IS BAD", filepath.c_str());
        return mat;
    }

    mat.reserve(num_boids);
    mat.resize(num_boids);
    for (auto& cell : mat)
    {
        cell.reserve(num_boids);
        cell.resize(num_boids);
    }

    for (size_t y = 0; y < num_boids; y++)
        for (size_t x = 0; x < num_boids; x++)
            mat[y][x] = 0;

    if (in.is_open())
    {
        std::string line = "";
        while (!in.eof()) 
        {
            std::getline(in, line);
            if (line.size() && !(line[0] >= '0' && line[0] <= '9'))
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
                    bool leftIsAssigned = left != -1;
                    while (!leftIsAssigned && i < line.size())
                    {
                        char c = line[i];
                        if (c == ':')
                            left = std::stoi(temp) - 1;
                        else if (c != ' ')
                            temp += c;
                        
                        leftIsAssigned = left != -1;
                        i++;
                    }
                }

                // stage 2
                {
                    temp = "";
                    while (i < line.size()) 
                    {
                        char c = line[i];
                        if (c == ' ' && temp != "")
                        {
                            right.push_back(std::stoi(temp) - 1);
                            temp = "";
                        }
                        else if (c != ' ')
                            temp += c;
                        
                        i++;
                    }

                    if (temp != "")
                    {
                        right.push_back(std::stoi(temp) - 1);
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

