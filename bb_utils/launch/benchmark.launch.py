from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import sys, os

# set launch arguments
bag_name = 'static_sensors_only'
for arg in sys.argv:
    if arg.startswith("bag_name:="):
        bag_name = str(arg.split(":=")[1])

def generate_launch_description():
    bag_folder = os.getenv('BAG_DIR') # set environmental variable BAG_DIR to the current directory where you save the bag files
    rviz_folder = os.getenv('RVIZ2_DIR') # set environmental variable RVIZ2_DIR to the current directory where you save the rviz2 config

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
        # remappings=[
        #     ("/bytetrack/detections", "/something"),
        #     ("/bytetrack/active_tracks", "/something")
        # ],
        parameters=[config],
        output='screen'
        # prefix=["xterm -font 10x20 -e"] # open in a new terminal with big font
    )


    ld = LaunchDescription([
        detect_and_track,
        benchmark_node
    ])

    return ld