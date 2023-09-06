import sys

# REMEMBER TO ADD THE FOLLOWING LINES TO NEW SCRIPTS
# TO IMPORT PYTHON MODULES
# import sys
# sys.path.append(configs_path)  # relative location of odt_configs
# from odt_configs import DIR_PATH1, DIR_PATH2, ...
# sys.path.append(DIR_PATH1)
# sys.path.append(DIR_PATH2)

########  PATH CONFIGS  ########################################################

# Update the ROOT_PATH to the parent directory location
ROOT_PATH = '/home/ale/big_brother/src/Big-Brother/'

# Update the DATA_PATH to the data directory location
DATA_PATH = '/home/ale/data'

DATASET_PATH = DATA_PATH + '/datasets'
OUTPUT_PATH = DATA_PATH + '/output'

TRACKING_PATH = ROOT_PATH + '/bb_tracker'
DETECTION_PATH = ROOT_PATH + '/bb_detection'
UTILS_PATH = ROOT_PATH + '/bb_utils'

### DETECTION

# VISTA
VISTA_PATH = DETECTION_PATH + '/vista'
VISTA_WORKDIR = OUTPUT_PATH + '/vista'

# Checklist for calibration files
INTRINSIC_PARAMS = [ 'fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'k3', 'p1', 'p2' ]
EXTRINSIC_PARAMS = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

# Sensors info
# https://www.mapix.com/wp-content/uploads/2018/07/97-0038_Rev-M_-HDL-32E_Datasheet_Web.pdf
LIDAR_RANGE_ACCURACY = 0.2       # [m] :
LIDAR_AZIMUTH_RESOLUTION = 0.4      # [deg] :
LIDAR_ANGULAR_RESOLUTION = 1.33     # [deg] :

##################################################################################
# Label local

coco_to_emot_class_mapping = { '0' : 2 }
nuscenes_to_emot_class_mapping = { '8' : 2, '7': 3 }
emot_label_mapping = {
    '1' : 'car',
    '2' : 'pedestrian',
    '3' : 'bicycle',
    '4' : 'bus',
    '5' : 'motorcycle',
    '6' : 'trailer',
    '7' : 'truck'
}

EMOT_LABEL_MAPPING_REV = {
    'car': '1',
    'pedestrian': '2',
    'bicycle': '3',
    'bus': '4',
    'motocycle': '5',
    'trailer': '6',
    'truck' : '7'
}

vista_label_mapping = {
    '0' : 'car',
    '1' : 'truck',
    '2' : 'construction_vehicle',
    '3' : 'bus',
    '4' : 'trailer',
    '5' : 'barrier',
    '6' : 'motocycle',
    '7' : 'bicycle',
    '8' : 'pedestrian',
    '9' : 'traffic_cone'
}

coco_label_mapping = {
    '0' : 'pedestrian',
    #...
}
class_colors = {
    'car' : (0, 255, 0),
    'pedestrian' : (255, 255, 0),
    'bicycle' : (255, 0, 0),
    'bus' : (0, 0, 255),
    'motorcycle' : (0, 255, 255),
    'trailer' : (255, 0, 255),
    'truck' : (128, 128, 128)
}

img_shape_zed2 = (480, 640, 3)

