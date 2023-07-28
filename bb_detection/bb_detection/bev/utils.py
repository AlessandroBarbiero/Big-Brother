#!/usr/bin/env python
from pathlib import Path
from configparser import SafeConfigParser
import codecs

def load_camera_data(data_path, intrinsic_file, extrinsic_file):
    ## Camera params (reading from files)
    camera_intrinsics_config_file = data_path  + intrinsic_file #'SN22892462.conf'
    camera_extrinsics_config_file = data_path + extrinsic_file #'extrinsics3.conf'
    camera_resolution = 'HD'
    camera_side = 'LEFT'
    config_intrinsics_section = camera_side + "_CAM_" + camera_resolution
    config_extrinsics_section = 'EXTRINSICS'
    # load intrinsics
    camera_config = SafeConfigParser()
    with codecs.open(camera_intrinsics_config_file, 'r', encoding="utf-8-sig") as f:
        camera_config.readfp(f)
    # load extrinsics
    with codecs.open(camera_extrinsics_config_file, 'r', encoding="utf-8-sig") as f:
        camera_config.readfp(f)
    camera_data = {
        'intrinsic': {
            'fx': camera_config.getfloat(config_intrinsics_section, 'fx'),
            'fy': camera_config.getfloat(config_intrinsics_section, 'fy'),
            'u0': camera_config.getfloat(config_intrinsics_section, 'cx'),
            'v0': camera_config.getfloat(config_intrinsics_section, 'cy')
            # TODO add distorsion!!
        },
        'extrinsic': {
            'x': camera_config.getfloat(config_extrinsics_section, 'x'),
            'y': camera_config.getfloat(config_extrinsics_section, 'y'),
            'z': camera_config.getfloat(config_extrinsics_section, 'z'),
            'yaw': camera_config.getfloat(config_extrinsics_section, 'yaw'),
            'pitch': camera_config.getfloat(config_extrinsics_section, 'pitch'),
            'roll': camera_config.getfloat(config_extrinsics_section, 'roll')
        },
        'center_of_mass': {
            # center of mass of the car
            'x': camera_config.getfloat(config_extrinsics_section, 'cm_x'),
            'y': camera_config.getfloat(config_extrinsics_section, 'cm_y'),
            'z': camera_config.getfloat(config_extrinsics_section, 'cm_z')
        }
    }
    return camera_data
