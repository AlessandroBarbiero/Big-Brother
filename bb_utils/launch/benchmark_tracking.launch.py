from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import sys, os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('bb_utils'),
        'config',
        'benchmark_params.yaml'
    )
    
    track_on_bag = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('bb_tracker'), 'launch'),
         '/track_on_bag.launch.py'])
      )

    benchmark_node = Node(
        package="bb_utils",
        executable="bb_benchmark",
        # remappings=[
        #     ("/bytetrack/detections", "/something"),
        #     ("/bytetrack/active_tracks", "/something")
        # ],
        parameters=[config],
        output='screen'
        # prefix=["xterm -font 10x20 -e"] # open in a new terminal with big font
    )

    visualizer_node = Node(
        package="bb_utils",
        executable="bb_visualizer",
        # prefix=["xterm -font 10x20 -e gdb -ex run --args"] # add gdb
    )


    ld = LaunchDescription([
        track_on_bag,
        benchmark_node,
        visualizer_node
    ])

    return ld