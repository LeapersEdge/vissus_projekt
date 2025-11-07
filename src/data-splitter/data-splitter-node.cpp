#include "data-splitter/data-splitter-node.hpp"
#include "vissus_projekt/msg/odometry_array.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Data_Splitter_Node>());
    rclcpp::shutdown();
    return 0;
}

void Data_Splitter_Node::Subscription_Odom_Callback(const Msg_Odom::SharedPtr odom, uint id)
{

}

