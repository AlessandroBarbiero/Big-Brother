from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('bb_tracker'),
        '.rviz2',
        'tracker.rviz'
    )

    config = os.path.join(
        get_package_share_directory('bb_tracker'),
        'config',
        'tracker_params.yaml'
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
    ld.add_action(rviz2_node)
    return ld