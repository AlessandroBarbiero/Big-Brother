from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

import sys, os

# set launch arguments
bag_name = 'static_sensors_only'
for arg in sys.argv:
    if arg.startswith("bag_name:="):
        bag_name = str(arg.split(":=")[1])

def generate_launch_description():
    bag_folder = os.getenv('BAG_DIR') # set environmental variable BAG_DIR to the current directory where you save the bag files

    emit_shutdown_action = Shutdown(reason='Bag play finished')

    detect = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('bb_detection'), 'detect.launch.py')])
      )

    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='static_sensors_only',
        description='Name of the bag you want to play'
    )

    # Start the bag slowing it by 10 times to allow the record 
    bag_process = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_folder + bag_name, '-r', '0.1'],
            on_exit=[
                emit_shutdown_action
                ],
        )
    
    register_bag = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', bag_folder + bag_name + '_det'], # '-s', 'mcap',
        )
    
    # Add a delay to allow Yolo to initialize the model
    delayed_bag = TimerAction(
        period=15.0, 
        actions=[
            register_bag,
            bag_process
            ]
        )


    ld = LaunchDescription([
        detect,
        bag_name_arg,
        delayed_bag,
    ])

    return ld