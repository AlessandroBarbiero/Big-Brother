import numpy as np
import os
import cv2
import glob

from utils import *
from Camera import Camera
from Bev import Bev

from os import listdir
from os.path import isfile, join

def compute_bev(event,x,y,flags,param):
    global bev_obj
    if event == cv2.EVENT_LBUTTONDBLCLK:
      print ("clicked point: "+str(x)+", "+str(y) )
      points = np.array([(x,y)])
      test_point = bev_obj.projectImagePointsToBevPoints(points)
      test_real_points = bev_obj.projectBevPointsToWorldGroundPlane(test_point)
      print ("bev point: "+str(test_real_points[0][0])+", "+str(test_real_points[0][1] ))

# Data params
in_data_path = 'data/' # path to all data files
in_video_path = in_data_path + 'frames/' # path to images folder
read_from_frame = 0
read_to_frame = -1 # -1: last frame found
# BEV params
bev_out_view = [0, 10, -20, 20] # [x_min, x_max, y_min, y_max] in meters
bev_out_image_width = 1000
bev_out_image_size = [np.nan, bev_out_image_width] # [height,width], only one needed, other nan

# build camera matrix
camera_data = load_camera_data(in_data_path, 'SN22892462.conf', 'indoor.conf')
camera = Camera(camera_data)
# init BEV object
bev_obj = Bev(camera, bev_out_view, bev_out_image_size)

# get files
onlyfiles = [f for f in listdir(in_video_path) if isfile(join(in_video_path, f))]
print (onlyfiles)

cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
cv2.resizeWindow("frame", 1280, 720)
    
cv2.setMouseCallback('frame',compute_bev)


for i in onlyfiles:
  frame = cv2.imread(in_video_path + i)  
  ret = (frame is not None)
   
  if ret:

    
    cv2.imshow("frame", frame)
    cv2.waitKey(0)


for i in onlyfiles:
  frame = cv2.imread(in_video_path + i)  
  ret = (frame is not None)
   
  if ret:

        # compute BEV
        bev = bev_obj.computeBev(frame)

        # Output
        cv2.namedWindow("Bev", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Bev", 1280, 720)
        cv2.imshow("Bev", bev)
        cv2.waitKey(0)

        
  else:
        print("Missed frame")
        # raise Exception("Missed frame")



