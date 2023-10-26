import numpy as np
import os
import cv2
import glob

from utils import *
from Camera import Camera
from Bev import Bev

# Data params
in_data_path = 'data/' # path to all data files
in_video_path = in_data_path + 'frames/' # path to images folder
read_from_frame = 0
read_to_frame = -1 # -1: last frame found
# BEV params
bev_out_view = [-2, 30, -20, 20] # [x_min, x_max, y_min, y_max] in meters
bev_out_image_width = 1000
bev_out_image_size = [np.nan, bev_out_image_width] # [height,width], only one needed, other nan

# build camera matrix
camera_data = load_camera_data(in_data_path, 'SN22892462.conf', 'extrinsics3.conf')
camera = Camera(camera_data)
# init BEV object
bev_obj = Bev(camera, bev_out_view, bev_out_image_size)

# setup input
video_in_folder_name = in_video_path.split('/')[-2]
frames_names = glob.glob1(in_video_path, '[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9].png') # get filename of all images in folder
frames_timestamps_full = np.sort(np.array([fn.replace('.png', '') for fn in frames_names]).astype(np.uint))
cap_fps = round(1 / np.mean(np.diff(frames_timestamps_full.astype(np.float) / 1e9)))
read_from_frame = read_from_frame
# support var for setting end frame
in_end_frame = read_to_frame if read_to_frame >= 0 else len(frames_names)

ret = True
frame_i = read_from_frame
while (frame_i < len(frames_names) and frame_i < in_end_frame and ret): # for each required frame in input video
    print("\nVideo: {} - Frame {}/{:.0f}".format(video_in_folder_name, frame_i+1, in_end_frame))

    # read new image frame
    frame_name = "{}.png".format(frames_timestamps_full[frame_i])
    frame = cv2.imread(in_video_path + frame_name)
    ret = (frame is not None)

    if ret:

        # compute BEV
        bev = bev_obj.computeBev(frame)

        # Output
        cv2.namedWindow("Bev", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Bev", 1280, 720)
        cv2.imshow("Bev", bev)
        cv2.waitKey(10)

        frame_i += 1
    else:
        print("Missed frame")
        # raise Exception("Missed frame")
