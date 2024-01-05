from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    thermal_cameras_topics = ["/carla/cross_s/thermal/",
                              "/carla/int_road_s/thermal/", 
                              "/carla/near_station_s/thermal/"
                              ]

    thermal_nodes = []
    for camera in thermal_cameras_topics:
        thermal_node = Node(
            package="bb_detection",
            executable="thermal_detector",
            remappings=[
                ("/to_detect", camera + "image"),
                ("/camera_info", camera + "camera_info"),
                ("/detection_2d", camera + "det2d")
            ],
            parameters=[
                {"show_debug": False},
                {"publish_3d": False},
                {"publish_2d": True},
                {"exclude_border_objects": False}
            ]
        )
        thermal_nodes.append(thermal_node)


    yolo_node = Node(
        package="bb_detection",
        executable="yolo_detector",
        parameters=[
            {"confidence_threshold": 0.1},
            {"show_debug": False},
            {"publish_3d": False},
            {"publish_2d": True},
            {"multi_topics": True},
            {"image_topic_list": [
                "/carla/cross_s/rgb/image",
                "/carla/int_road_s/rgb/image",
                "/carla/near_station_s/rgb_w_station/image",
                "/carla/near_station_s/rgb_w_street/image",
                ]
            }
        ]
    )

    lidar_node = Node(
        package="bb_detection",
        executable="lidar_detector",
        remappings=[
            ("/lidar", "/carla/cross_s/lidar"),
            ("/detection_3d", "/bytetrack/detections3d")
        ],
        parameters=[
            {'lidar_list'           : [
                    "cross_s/lidar", 
                    "near_station_s/lidar", 
                    "int_road_s/lidar"
                ]
            },
            {'lidar_max_distances'  : [30, 30, 30]},
            {"fixed_frame"          : "map"},
            {"random_seed"          : 42},
            {"percentage_miss"      : 0.15},
            {"noise_position"       : 0.15},
            {"noise_size"           : 0.30},
            {"noise_orientation"    : 0.0}
        ]
    )


    ld = LaunchDescription()
    for thermal_node in thermal_nodes:
        ld.add_action(thermal_node)
    ld.add_action(yolo_node)
    ld.add_action(lidar_node)
    return ld