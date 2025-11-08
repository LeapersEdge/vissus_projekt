from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # === Launch Arguments ===
    boid_num_arg = DeclareLaunchArgument(
        "boid_number",
        default_value="3",  # Change this default if needed
        description="Number of boid_controller nodes to launch"
    )

    boid_number = LaunchConfiguration("boid_number")

    vissus_dir = get_package_share_directory("vissus_projekt")
    start_launch_path = os.path.join(vissus_dir, "launch", "start.launch.py")

    start_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(start_launch_path)
    )

    boid_controllers = []
    for i in range(boid_number):
        boid_controllers.append(
            Node(
                package="vissus_projekt",
                executable="boid-controller",
                name=f"boid_controller_{i}",
                output="screen",
                parameters=[{"robot_id": i}]
            )
        )

    data_splitter = Node(
        package="vissus_projekt",
        executable="data-splitter",
        name="data_splitter",
        output="screen",
        parameters=[{"boid_number": boid_number}]
    )

    # === Build Launch Description ===
    ld = LaunchDescription()
    ld.add_action(boid_number_arg)
    ld.add_action(start_launch)

    for controller in boid_controllers:
        ld.add_action(controller)

    ld.add_action(data_splitter)

    return ld
