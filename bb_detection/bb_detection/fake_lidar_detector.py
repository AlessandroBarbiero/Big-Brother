# ROS
import rclpy
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger as Logger
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_pose
# ROS messages
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose, Detection3DArray, BoundingBox3D
from geometry_msgs.msg import TransformStamped, Point
# Python
from typing import List
import numpy as np
import random

from .utility import classes_to_detect

class FakeDetector(Node):

    def __init__(self):
        super().__init__('fake_lidar_detector')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('lidar_list', ["/lidar_topic"]),
                ('lidar_max_distances', [30]),
                ("fixed_frame", "/map"),
                ("percentage_miss", 0.15),
                ("noise_position", 0.01),
                ("noise_size", 0.01),
                ("noise_orientation", 0.001)
            ]
        )

        self.lidar_list = self.get_parameter('lidar_list').value
        self.lidar_max_distances = self.get_parameter('lidar_max_distances').value
        self.fixed_frame = self.get_parameter('fixed_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lidar_ready = False

        self._pub_detections = self.create_publisher(Detection3DArray, 'detection_3d', 10)
        self._sub_pc = self.create_subscription(PointCloud2, "lidar", self.callback_pc, 10)

        # Subscribe to ground truth
        trans_local_qos = QoSProfile(depth=10,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self._sub_gt_static = self.create_subscription(MarkerArray, "carla/markers/static", self.callback_gt_static, qos_profile=trans_local_qos)
        self._sub_gt_moving = self.create_subscription(MarkerArray, "carla/markers", self.callback_gt_moving, 10)
        # Init a variable to false, it turns true every time a point cloud message is received,
        # signaling that it's time to publish the fake detections
        self.need_to_pub = False

    def callback_gt_static(self, msg : MarkerArray):
        self.static_objects = msg.markers
        self.get_logger().info("Static Objects saved!")
        return
    
    def callback_gt_moving(self, msg : MarkerArray):
        self.moving_objects = msg.markers
        if self.need_to_pub == True:
            self.publish_detections(msg.markers[0].header.stamp)
            self.need_to_pub = False
        return
    
    def init_lidar_tf(self):
        '''
        Update self.last_transform_lidar with the last transformations that brings
        the lidars tf to the fixed frame
        '''
        self.last_transform_lidar = []
        for lidar_topic in self.lidar_list:
            try:
                tf : TransformStamped = self.tf_buffer.lookup_transform(
                    lidar_topic,
                    self.fixed_frame,
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(f'Could not transform {lidar_topic} to {self.fixed_frame}: {ex}')
                return

            self.last_transform_lidar.append(tf)
        self.lidar_ready = True

    def filter_lidar(self, objects : List[Marker]):
        '''
        Return a filtered list of objects containing only objects visible from the lidars
        '''
        def distance_from_origin(position : Point):
            return math.sqrt(position.x**2 + position.y**2 + position.z**2)
        
        filtered = []
        for obj in objects:
            lidar_tf : TransformStamped
            for i, lidar_tf in enumerate(self.last_transform_lidar):
                if distance_from_origin(                                                                \
                     do_transform_pose(pose=obj.pose, transform=lidar_tf).position) \
                       < self.lidar_max_distances[i]:
                    filtered.append(obj)
                    break  # Go to next object

        return filtered

    def publish_detections(self, stamp):
        detections_msg = Detection3DArray()
        detections_msg.header.stamp = stamp
        detections_msg.header.frame_id = self.fixed_frame
  
        all_objects = self.static_objects + self.moving_objects  
        seen_objects = self.filter_lidar(all_objects)

        percentage_miss = self.get_parameter('percentage_miss').value
        noise_position = self.get_parameter('noise_position').value
        noise_size = self.get_parameter('noise_size').value
        noise_orient = self.get_parameter('noise_orientation').value

        marker : Marker
        for marker in seen_objects:
            rnd_miss = random.uniform(0,1)
            if(rnd_miss<percentage_miss):
                continue
            rnd_pos =       [random.uniform(-noise_position, noise_position) for _ in range(0,3)]
            rnd_size =      [random.uniform(-noise_size, noise_size) for _ in range(0,3)]
            rnd_orient =    [random.uniform(-noise_orient, noise_orient) for _ in range(0,4)]

            detection_a = Detection3D()
            detection_a.header = detections_msg.header
            # create hypothesis
            hypothesis = ObjectHypothesisWithPose()

            if marker.ns.lower() in classes_to_detect:
                hypothesis.id = marker.ns.lower()
            else:
                if marker.scale.x>1.0 or marker.scale.y>1.0:
                    hypothesis.id = "car"
                else:
                    hypothesis.id = "person"
            hypothesis.score = 0.9

            detection_a.results.append(hypothesis)

            detection_a.bbox.center = marker.pose
            detection_a.bbox.center.position.x += rnd_pos[0]
            detection_a.bbox.center.position.y += rnd_pos[1]
            detection_a.bbox.center.position.z += rnd_pos[2]
            detection_a.bbox.center.orientation.x += rnd_orient[0]
            detection_a.bbox.center.orientation.y += rnd_orient[1]
            detection_a.bbox.center.orientation.z += rnd_orient[2]
            detection_a.bbox.center.orientation.w += rnd_orient[3]

            detection_a.bbox.size = marker.scale
            detection_a.bbox.size.x += rnd_size[0]
            detection_a.bbox.size.y += rnd_size[1]
            detection_a.bbox.size.z += rnd_size[2]

            detections_msg.detections.append(detection_a)

        self._pub_detections.publish(detections_msg)
        # self.get_logger().info("Published msg: " + str(msg.header.stamp.sec) + "." + str(msg.header.stamp.nanosec))


    def callback_pc(self, msg : PointCloud2):
        if not self.lidar_ready:
            self.init_lidar_tf()
            if not self.lidar_ready: return

        self.need_to_pub = True



## -----------------------------------------------------------
## The code below is "ported" from 
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
import sys
from collections import namedtuple
import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

## End porting ------------------






def main(args=None):
    rclpy.init(args=args)

    Logger('LOG').info("Initiatizing node...")

    detector = FakeDetector()

    try:
        rclpy.spin(detector)
    except:
        print("Lidar Detector Terminated")


    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    