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

    yolo_node = Node(
    package="bb_detection",
    executable="yolo_detector",
    remappings=[
        ("/to_detect", "/carla/sensors_home/static_rgb_camera/image")
    ],
    parameters=[
        {"confidence_threshold": 0.5},
        {"show_debug": False}
    ]
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

    ld = LaunchDescription()
    ld.add_action(bag_name_arg)
    ld.add_action(yolo_node)
    ld.add_action(bag_process)
    return ld