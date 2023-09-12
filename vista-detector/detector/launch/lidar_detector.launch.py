from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lidar_node = Node(
        package="detector",
        executable="lidar_detector",
        remappings=[
            ("/lidar", "/carla/sensors_home/static_lidar"),
        ],
        output='screen'
    )

    static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['2.6', '21.95', '0.15', '0', '0', '0.99688761', '0.07883583', 'map', 'sensors_home']
        )

    ld = LaunchDescription()
    ld.add_action(static_tf)
    ld.add_action(lidar_node)
    return ld