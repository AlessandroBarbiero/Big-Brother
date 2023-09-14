import sys
# sys.path.append('../config')
from .config.odt_configs import VISTA_PATH, VISTA_WORKDIR, \
                        OUTPUT_PATH, vista_label_mapping
import os
import time
import numpy as np
import torch
from .vista.det3d.models import build_detector
from .vista.det3d.torchie import Config
from .vista.det3d.torchie.apis import (
    batch_processor,
    build_optimizer,
    get_root_logger,
    init_dist,
    set_random_seed,
    train_detector,
)
from .vista.det3d.torchie.trainer import get_dist_info, load_checkpoint
import warnings

from .vista.det3d.utils.config_tool import get_downsample_factor
from .vista.det3d.core.input.voxel_generator import VoxelGenerator
from .vista.det3d.core.anchor.target_assigner import TargetAssigner
import torch.distributed

from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger as Logger
from sensor_msgs.msg import PointCloud2
# Ported from rosmot to Detection3DArray
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose, Detection3DArray

logTimes = False
showTestCar = False


warnings.filterwarnings("ignore")

default_config_file = VISTA_PATH + "/configs/vista/vista.py"
default_work_dir = OUTPUT_PATH + '/work_dir'
default_checkpoint_file = VISTA_WORKDIR + "/checkpoints/vista.pth"

cum_input_time = 0.0
cum_inputproc_time = 0.0
cum_network_time = 0.0
cum_output_time = 0.0
cum_total_time = 0.0


class VISTADetector(Node):

    def __init__(self,
                 config_file=default_config_file,
                 work_dir=default_work_dir,
                 checkpoint_file=default_checkpoint_file,
                 local_rank=0,
                 gpus=1
                 ):
        super().__init__('lidar_detector')

        cfg = Config.fromfile(config_file)
        cfg.local_rank = local_rank

        self.num_frame = 0
        
        # Time info
        self.run_info = {}
        self.run_info['input_time'] = 0.0        # time to read pc
        self.run_info['inputproc_time'] = 0.0    # time for preprocessing (voxelization)
        self.run_info['network_time'] = 0.0      # network time 
        self.run_info['output_time'] = 0.0       # output time
        self.run_info['total_time'] = 0.0        # total time

        self.run_info['avg_input_time'] = 0.0        
        self.run_info['avg_inputproc_time'] = 0.0           
        self.run_info['avg_network_time'] = 0.0             
        self.run_info['avg_output_time'] = 0.0       
        self.run_info['avg_total_time'] = 0.0  

        self.get_logger().info("Configuration object created")

        # update local according to CLI args
        if work_dir is not None:
            cfg.work_dir = work_dir

        distributed = False
        if "WORLD_SIZE" in os.environ:
            distributed = int(os.environ["WORLD_SIZE"]) > 1

        if distributed:
            torch.cuda.set_device(local_rank)
            torch.distributed.init_process_group(
                backend="nccl", init_method="env://")

            cfg.gpus = torch.distributed.get_world_size()
        else:
            cfg.gpus = gpus

        self.get_logger().info("Setting the device...")

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # init logger before other steps
        self.get_logger().info("Distributed testing: {}".format(distributed))
        self.get_logger().info(f"torch.backends.cudnn.benchmark: {torch.backends.cudnn.benchmark}")

        #breakpoint()
        model = build_detector(cfg.model, train_cfg=None, test_cfg=cfg.test_cfg).to(device).float().eval()

        checkpoint = load_checkpoint(model, checkpoint_file, map_location="cpu")

        self.net = model
        self.target_assigner = cfg.target_assigner
        self.voxel_gen_dict = cfg.voxel_generator

        self.get_logger().info('network done, voxel done.')



        self.voxel_generator = VoxelGenerator(
            voxel_size=self.voxel_gen_dict.voxel_size,
            point_cloud_range=self.voxel_gen_dict.range,
            max_num_points=self.voxel_gen_dict.max_points_in_voxel,
            max_voxels=self.voxel_gen_dict.max_voxel_num,
            faster=True#self.voxel_gen_dict.faster
        )

        feature_map_size = self.voxel_generator.grid_size[:2] // get_downsample_factor(cfg.model)
        feature_map_size = [*feature_map_size, 1][::-1]

        self._pub = self.create_publisher(Detection3DArray, 'detection_3d', 10)
        self._sub = self.create_subscription(PointCloud2, "/lidar", self.callback, 10)


    def load_example_from_points(self, points):
        
        global cum_inputproc_time 

        inputproc_start_time = time.time()
        
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        points = points[points[:, 4].argsort()]
        res = self.voxel_generator.generate(points, max_voxels=self.voxel_gen_dict.max_voxel_num)
        voxels, coords, num_points = res[0], res[1], res[2]
        coords = np.pad(coords, ((0, 0), (1, 0)), mode='constant', constant_values=0)
        voxels = torch.tensor(voxels, dtype=torch.float32).to(device, non_blocking=False)
        coords = torch.tensor(coords.copy(), dtype=torch.int32).to(device, non_blocking=False)
        num_points = torch.tensor(num_points, dtype=torch.int32).to(device, non_blocking=False)
        num_voxels = torch.tensor([voxels.shape[0]], dtype=torch.int64).to(device, non_blocking=False)
        
        self.run_info['inputproc_time'] = time.time() - inputproc_start_time 
        cum_inputproc_time += self.run_info['inputproc_time']
       
        res = {
            #'anchors': self.anchors,
            'voxels': voxels,
            'num_points': num_points,
            'coordinates': coords,
            'num_voxels': num_voxels,
            'shape': [self.voxel_generator.grid_size],
            'annos': []
        }

        return res


    def callback(self, msg : PointCloud2):

        global cum_input_time, cum_inputproc_time
        global cum_network_time, cum_output_time, cum_total_time

        self.num_frame += 1

        input_start_time = time.time()
        
        points_raw = list(read_points(msg, skip_nans=False, field_names=("y", "x",  "z", "intensity", "ring")))
        points = np.array(list(points_raw), dtype=np.float32)
        self.run_info['input_time'] = time.time() - input_start_time
        cum_input_time += self.run_info['input_time']

        self.get_logger().info("shape{}".format(np.shape(points)))
        
        # points[:, 3] /= 255            #Uncomment for data from VLP16
        # points[:,4] = 0                #Uncomment for data from VLP16
        sweep_points_list = [points]
        # points = np.concatenate(sweep_points_list, axis=0)[:, [0, 1, 2, 4]]    #Uncomment for data from VLP16
        # points = np.concatenate(sweep_points_list, axis=0)[:, [0, 1, 2]]  # Comment for data from VLP16

        # Add all zeros as the ring parameter
        Temp = np.zeros(points.shape[0])  # Comment for data from VLP16
        points = np.c_[points, Temp]  # Comment for data from VLP16

        
        self.get_logger().info("shape: {}".format(np.shape(points)))

        self.inference_model(points, msg)
        
        

    def inference_model(self, points, msg : PointCloud2):

        global cum_network_time, cum_output_time
        global cum_input_time, cum_inputproc_time, cum_total_time
       
        network_start_time = time.time()
        
        example = self.load_example_from_points(points)

        pred = self.net(example, return_loss=False)[0]
        self.run_info['network_time'] = time.time() - network_start_time
        cum_network_time += self.run_info['network_time']
                
       
        output_start_time = time.time()
        
        box3d = pred['box3d_lidar'].detach().cpu().numpy()
        locs = box3d[:, :3]
        dims = box3d[:, 3:6]
        #rots = np.concatenate([np.zeros([locs.shape[0], 2], dtype=np.float32), box3d[:, 8:]], axis=1)
        rots = -box3d[:, -1] - np.pi / 2
        vel = box3d[:, 6:8].round(decimals=3)
        label = pred["label_preds"].detach().cpu().numpy().tolist()
        score = pred["scores"].detach().cpu().numpy().tolist()

        detections_msg = Detection3DArray()
        detections_msg.header = msg.header

        index = 0

        labels_to_keep = [0,6,7,8]
        while (index < len(score)):  # minimum confidence
            if (score[index] > 0.3 and label[index] in labels_to_keep):
                
                # Possibility to have also a linear velocity
                # twist = Twist()
                # twist.linear.x = vel[index, 0]
                # twist.linear.y = vel[index, 1]
                # twist.linear.z = 0.0
                # twist.angular.x = 0.0
                # twist.angular.y = 0.0
                # twist.angular.z = 0.0
                detection_a = Detection3D()
                detection_a.header = msg.header

                # create hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = vista_label_mapping[str(label[index])]
                hypothesis.score = score[index]
                detection_a.results.append(hypothesis)

                rot_q = quaternion_from_euler(0.0, #rots[index, 0], 
                                              0.0, #rots[index, 1], 
                                              rots[index])
                detection_a.bbox.center.orientation.x = rot_q[0] #0
                detection_a.bbox.center.orientation.y = rot_q[1] #0
                detection_a.bbox.center.orientation.z = rot_q[2] #np.sin(rots[index] / 2)[2]
                detection_a.bbox.center.orientation.w = rot_q[3] #np.cos(rots[index] / 2)[2]

                detection_a.bbox.center.position.x = float(locs[index, 0])
                detection_a.bbox.center.position.y = float(locs[index, 1])
                detection_a.bbox.center.position.z = float(locs[index, 2])

                detection_a.bbox.size.x = float(dims[index, 0])
                detection_a.bbox.size.y = float(dims[index, 1])
                detection_a.bbox.size.z = float(dims[index, 2])

                # Old comments
                #box_a.dimensions.x = dims[index, 1]
                #box_a.dimensions.y = dims[index, 0]
                #box_a.pose.orientation.z = np.sin((rots[index][2]-np.pi/2)/2)
                #box_a.pose.orientation.w = np.cos((rots[index][2]-np.pi/2)/2)
                detections_msg.detections.append(detection_a)

            index = index + 1
        
        
        self.run_info['output_time'] = time.time() - output_start_time
        cum_output_time += self.run_info['output_time']
        
        self.run_info['total_time'] = self.run_info['input_time'] + \
                                      self.run_info['inputproc_time'] + \
                                      self.run_info['network_time'] + \
                                      self.run_info['output_time'] 
        cum_total_time += self.run_info['total_time']


        self.get_logger().debug(f"Input time : {self.run_info['input_time']: .3f} sec, {(self.run_info['input_time']/self.run_info['total_time'])*100: .1f}% of total time")
        self.get_logger().debug(f"Input processing time : {self.run_info['inputproc_time']: .3f}, {(self.run_info['inputproc_time']/self.run_info['total_time'])*100: .1f}% of total time")
        self.get_logger().debug(f"Network time : {self.run_info['network_time']: .3f} sec, {(self.run_info['network_time']/self.run_info['total_time'])*100: .1f}% of total time")
        self.get_logger().debug(f"Output processing time : {self.run_info['output_time']: .3f} sec, {(self.run_info['output_time']/self.run_info['total_time'])*100: .1f}% of total time")
        self.get_logger().debug(f"Total inference time : {self.run_info['total_time']: .3f} sec") 

        
        self.run_info['avg_output_time'] = cum_output_time / self.num_frame
        self.run_info['avg_network_time'] = cum_network_time / self.num_frame
        self.run_info['avg_input_time'] = cum_input_time / self.num_frame
        self.run_info['avg_inputproc_time'] = cum_inputproc_time / self.num_frame
        self.run_info['avg_total_time'] = cum_total_time / self.num_frame

        print()
        self.get_logger().debug(f"Average input time : {self.run_info['avg_input_time']: .3f} sec, {(self.run_info['avg_input_time']/self.run_info['avg_total_time'])*100: .1f}% of total time")
        self.get_logger().debug(f"Average input processing time : {self.run_info['avg_inputproc_time']: .3f}, {(self.run_info['avg_inputproc_time']/self.run_info['avg_total_time'])*100: .1f}% of total time")
        self.get_logger().debug(f"Average network time : {self.run_info['avg_network_time']: .3f} sec, {(self.run_info['avg_network_time']/self.run_info['avg_total_time'])*100: .1f}% of total time")
        self.get_logger().debug(f"Average output processing time : {self.run_info['avg_output_time']: .3f} sec, {(self.run_info['avg_output_time']/self.run_info['avg_total_time'])*100: .1f}% of total time")
        self.get_logger().debug(f"Average total inference time : {self.run_info['total_time']: .3f} sec")


        self._pub.publish(detections_msg)
        self.get_logger().info("Published msg: " + str(msg.header.stamp.sec) + "." + str(msg.header.stamp.nanosec))


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

    config_file = default_config_file
    checkpoint_file = default_checkpoint_file

    Logger('LOG').info("Building the network with the following parameters: \n")
    Logger('LOG').info("CHECKPOINT : " + checkpoint_file)
    Logger('LOG').info("CONFIG FILE : " + config_file)

    detector = VISTADetector(config_file, checkpoint_file)

    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    