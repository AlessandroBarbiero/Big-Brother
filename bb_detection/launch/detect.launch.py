from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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
            {"publish_2d": True},
            {"exclude_border_objects": False}
        ]
    )

    yolo_node = Node(
        package="bb_detection",
        executable="yolo_detector",
        remappings=[
            ("/to_detect", "/carla/sensors_home/static_rgb_camera/image"),
            ("/camera_info", "/carla/sensors_home/static_rgb_camera/camera_info"),
            # ("/depth", "/carla/sensors_home/static_depth_camera/image"),
            # ("/detection_3d", "/bytetrack/detections3d"),
            ("/detection_2d", "/carla/sensors_home/static_rgb_camera/det2d")
        ],
        parameters=[
            {"confidence_threshold": 0.1},
            {"show_debug": False},
            {"publish_3d": False},
            {"publish_2d": True},
            {"multi_topics": True},
            {"image_topic_list": [
                "/carla/sensors_home/static_rgb_camera/image"
                ]
            }
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

    # static_tf = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments = ['0', '0', '0', '-0.5', '0.5', '-0.5', '-0.5', 'sensors_home/static_termic_camera', 'sensors_home/sensors_frame']
    #     )
    # static_tf_2 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = ['0', '0', '0', '0', '0', '0', '1', 'sensors_home/static_lidar', 'sensors_home']
    # )

    ld = LaunchDescription()
    ld.add_action(thermal_node)
    ld.add_action(yolo_node)
    ld.add_action(lidar_node)
    # ld.add_action(static_tf)
    # ld.add_action(static_tf_2)
    return ld