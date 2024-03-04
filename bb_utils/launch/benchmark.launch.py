from launch import LaunchDescription
from launch_ros.actions import Node
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

    detect_and_track = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('bb_tracker'), 'launch'),
         '/detect_and_track.launch.py'])
      )

    benchmark_node = Node(
        package="bb_utils",
        executable="bb_benchmark",
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
        detect_and_track,
        benchmark_node,
        visualizer_node
    ])

    return ld