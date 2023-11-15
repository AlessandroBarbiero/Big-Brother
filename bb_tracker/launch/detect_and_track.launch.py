from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
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

    thermal_node = Node(
        package="bb_detection",
        executable="thermal_detector",
        remappings=[
            ("/to_detect", "/carla/sensors_home/static_termic_camera/image"),
            ("/camera_info", "/carla/sensors_home/static_termic_camera/camera_info"),
            # ("/detection_3d", "/bytetrack/detections3d"),
            ("/detection_2d", "/carla/sensors_home/static_termic_camera/det2d")
        ],
        parameters=[
            {"show_debug": False},
            {"publish_3d": False},
            {"publish_2d": True}
        ]
    )

    yolo_node = Node(
        package="bb_detection",
        executable="yolo_detector",
        remappings=[
            ("/to_detect", "/carla/sensors_home/static_rgb_camera/image"),
            ("/camera_info", "/carla/sensors_home/static_rgb_camera/camera_info"),
            ("/depth", "/carla/sensors_home/static_depth_camera/image"),
            # ("/detection_3d", "/bytetrack/detections3d"),
            ("/detection_2d", "/carla/sensors_home/static_rgb_camera/det2d")
        ],
        parameters=[
            {"confidence_threshold": 0.1},
            {"show_debug": False},
            {"publish_3d": False},
            {"publish_2d": True}
        ]
    )

    lidar_node = Node(
        package="bb_detection",
        executable="lidar_detector",
        remappings=[
            ("/lidar", "/carla/sensors_home/static_lidar"),
            ("/detection_3d", "/bytetrack/detections3d")
        ],
        parameters=[
            {'lidar_list'           : ["sensors_home/static_lidar"]},
            {'lidar_max_distances'  : [30]},
            {"fixed_frame"          : "map"},
            {"random_seed"          : 42},
            {"percentage_miss"      : 0.15},
            {"noise_position"       : 0.15},
            {"noise_size"           : 0.30},
            {"noise_orientation"    : 0.0}
        ]
    )

    static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '-0.5', '0.5', '-0.5', '-0.5', 'sensors_home/static_termic_camera', 'sensors_home/sensors_frame']
        )
    static_tf_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', '1', 'sensors_home/static_lidar', 'sensors_home']
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
            arguments=['-d', [os.path.join(rviz_folder, 'tracker.rviz')]],
            output='log',
            prefix=["xterm -e"] # open in a new terminal
        )

    ld = LaunchDescription()
    ld.add_action(tracker_node)
    ld.add_action(bag_name_arg)
    ld.add_action(thermal_node)
    # ld.add_action(yolo_node)
    # ld.add_action(lidar_node)
    ld.add_action(static_tf)
    ld.add_action(static_tf_2)
    ld.add_action(bag_process)
    ld.add_action(rviz2_node)
    return ld