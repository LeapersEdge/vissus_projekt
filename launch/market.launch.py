import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    auctioneer_node = Node(
        package='vissus_projekt',
        executable='auctioneer.py',
        name='auctioneer',
        output='screen'
    )
    ld.add_action(auctioneer_node)

    # TODO: everything here
    num_robots = 4
    for i in range(num_robots):
        bidder_node = Node(
            package='vissus_projekt',
            executable='bidder.py',
            name=f'bidder_{i}',
            parameters=[{
                'robot_id': i,
                'start_x': -0.5 if i%2 == 0 else 0.5,
                'start_y': -4.5 if i>1 else -4.0
            }],
            output='screen'
        )
        ld.add_action(bidder_node)

    return ld
