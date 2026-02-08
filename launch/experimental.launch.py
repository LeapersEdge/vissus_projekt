from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    launch_dir = get_package_share_directory('vissus_projekt')
    yaml_path = os.path.join(launch_dir, 'launch', 'launch_params.yaml')

    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)

    num_of_robots = int(params['num_of_robots'])
    boid_vision_range = float(params['boid_vision_range'])
    boid_fov = float(params['boid_fov'])
    mode = params['mode']

    boid_controllers = []
    for i in range(1, num_of_robots+1):
        boid_controllers.append(
            Node(
                package='vissus_projekt',
                executable='boid_control',
                name=f'boid_controller_{i}',
                output='screen',
                parameters=[{'robot_id': int(i)}, {'mode': mode}]
            )
        )

    data_splitter = Node(
        package='vissus_projekt',
        executable='data_splitter',
        name='data_splitter',
        output='screen',
        parameters=[{'boid_number': num_of_robots}, {'boid_fov': boid_fov}, {'boid_vision_range': boid_vision_range}]
    )

    return LaunchDescription(boid_controllers + [data_splitter])

