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
ROOT_PATH = '/ros2_ws/src/'

# Update the DATA_PATH to the data directory location
DATA_PATH = '/home'
OUTPUT_PATH = DATA_PATH + '/output'

DETECTION_PATH = ROOT_PATH + '/detector/detector'

### DETECTION

# VISTA
VISTA_PATH = DETECTION_PATH + '/vista'
VISTA_WORKDIR = DATA_PATH + '/vista'

##################################################################################
# Label local

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

class_colors = {
    'car' : (0, 255, 0),
    'pedestrian' : (255, 255, 0),
    'bicycle' : (255, 0, 0),
    'bus' : (0, 0, 255),
    'motorcycle' : (0, 255, 255),
    'trailer' : (255, 0, 255),
    'truck' : (128, 128, 128)
}


