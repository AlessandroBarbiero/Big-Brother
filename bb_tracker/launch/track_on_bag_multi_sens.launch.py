from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

import sys, os
import math, json
from transforms3d.euler import euler2quat

# set launch arguments
bag_name = 'static_sensors_only_det'
for arg in sys.argv:
    if arg.startswith("bag_name:="):
        bag_name = str(arg.split(":=")[1])


CARLA_CONFIG_DIR = r'/home/ale/carla-ros-bridge/src/ros-bridge/carla_ros_bridge/config'

def carla_rotation_to_RPY(roll, pitch, yaw):
    """
    Convert a carla rotation to a roll, pitch, yaw tuple

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    """
    _roll = math.radians(roll)
    _pitch = -math.radians(pitch)
    _yaw = -math.radians(yaw)

    return (_roll, _pitch, _yaw)

def carla_rotation_to_quat(roll, pitch, yaw):
    _roll, _pitch, _yaw = carla_rotation_to_RPY(roll=roll, pitch=pitch, yaw=yaw)
    quat = euler2quat(_roll, _pitch, _yaw)
    return quat

# --------------------------

def generate_launch_description():
    bag_folder = os.getenv('BAG_DIR') # set environmental variable BAG_DIR to the current directory where you save the bag files

    rviz_config_file = os.path.join(
        get_package_share_directory('bb_tracker'),
        '.rviz2',
        'tracker.rviz'
    )

    config_file = os.path.join(
        get_package_share_directory('bb_tracker'),
        'config',
        'tracker_params_multi_sens.yaml'
    )

    # Static TF -------------------------
    with open(CARLA_CONFIG_DIR + '/sensors_homes.json') as f:
        sensors_config = json.load(f)
    sensors_home_actors = sensors_config["actors"]

    static_tf_nodes = []
    for sh in sensors_home_actors:
        location = sh["location"]
        x = str(location["x"])
        y = str(-location["y"]) # Invert y because we are passing from left to right hand system
        z = str(location["z"])
        rotation = sh["rotation"]
        quat = carla_rotation_to_quat(**rotation)
        qw = str(quat[0])
        qx = str(quat[1])
        qy = str(quat[2])
        qz = str(quat[3])

        static_tf_nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = [x,y,z,   qx, qy, qz, qw, 'map', sh["name"]]
            )
        )

    # -------------------------------

    tracker_node = Node(
        package="bb_tracker",
        executable="bb_tracker",
        parameters=[config_file],
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
            cmd=['ros2', 'bag', 'play', bag_folder + bag_name, '--read-ahead-queue-size', '10000'], #, '-r', '0.9'],
        )
    
    rviz2_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='log',
            prefix=["xterm -e"] # open in a new terminal
        )

    ld = LaunchDescription()
    for n in static_tf_nodes:
        ld.add_action(n)
    ld.add_action(tracker_node)
    ld.add_action(bag_name_arg)
    ld.add_action(bag_process)
    ld.add_action(rviz2_node)
    return ld