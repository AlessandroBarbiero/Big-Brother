from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

import sys, os

# set launch arguments
bag_name = 'static_sensors_only_det'
for arg in sys.argv:
    if arg.startswith("bag_name:="):
        bag_name = str(arg.split(":=")[1])

def generate_launch_description():
    bag_folder = os.getenv('BAG_DIR') # set environmental variable BAG_DIR to the current directory where you save the bag files

    rviz_config = os.path.join(
        get_package_share_directory('bb_tracker'),
        '.rviz2',
        'tracker.rviz'
    )

    config = os.path.join(
        get_package_share_directory('bb_tracker'),
        'config',
        'tracker_params_no_det.yaml'
    )

    tracker_node = Node(
        package="bb_tracker",
        executable="bb_tracker",
        remappings=[
            # ("/bytetrack/camera_info", "/carla/sensors_home/static_termic_camera/camera_info")
        ],
        parameters=[config],
        output='screen',
        #prefix=["xterm -font 10x20 -g 100x25 -e gdb -ex run --args"]          # add gdb
        #prefix=["xterm -font 10x20 -g 100x25 -e gdbserver localhost:3100"]    # add gdb-server to use with vsCode
        prefix=["xterm -font 10x20 -g 100x25 -e"]                              # open in a new terminal with big font
    )

    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='static_sensors_only',
        description='Name of the bag you want to play'
    )

    bag_process = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_folder + bag_name], #, '-r', '0.9'],
        )
    
    rviz2_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='log',
            prefix=["xterm -e"] # open in a new terminal
        )

    ld = LaunchDescription()
    ld.add_action(tracker_node)
    ld.add_action(bag_name_arg)
    ld.add_action(bag_process)
    ld.add_action(rviz2_node)
    return ld