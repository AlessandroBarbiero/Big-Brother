from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument

import sys, os

# set launch arguments
bag_name = 'static_sensors_only'
for arg in sys.argv:
    if arg.startswith("bag_name:="):
        bag_name = str(arg.split(":=")[1])

def generate_launch_description():
    bag_folder = os.getenv('BAG_DIR') # set environmental variable BAG_DIR to the current directory where you save the bag files
    rviz_folder = os.getenv('RVIZ2_DIR') # set environmental variable RVIZ2_DIR to the current directory where you save the rviz2 config

    thermal_node = Node(
        package="bb_detection",
        executable="thermal_detector",
        remappings=[
            ("/to_detect", "/carla/sensors_home/static_termic_camera/image"),
            ("/camera_info", "/carla/sensors_home/static_termic_camera/camera_info")
        ],
        parameters=[
            {"show_debug": True},
            {"exclude_border_objects": False}
        ]
    )

    static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '-0.5', '0.5', '-0.5', '-0.5', 'sensors_home/static_termic_camera', 'sensors_home/sensors_frame']
        )

    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='static_sensors_only',
        description='Name of the bag you want to play'
    )

    bag_process = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_folder + bag_name],
            # output='screen'
        )
    
    rviz2_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(rviz_folder, 'static_sensors.rviz')]],
            prefix=["xterm -e"] # open in a new terminal
        )

    ld = LaunchDescription()
    ld.add_action(bag_name_arg)
    ld.add_action(thermal_node)
    ld.add_action(static_tf)
    ld.add_action(bag_process)
    ld.add_action(rviz2_node)
    return ld