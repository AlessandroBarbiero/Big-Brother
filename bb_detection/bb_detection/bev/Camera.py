import numpy as np
import cv2
from math import cos, sin, pi

class Camera:
    """
    Describes a monocular camera. It stores it parameters and performs projection operations between world frame,
    camera frame and image place.
    Notice that no distorsion is now considered, neither in storing the parameters nor in projecting points.
    """

    def __init__(self, cameraData):
        """
        Initializes the camera object with its intrinsic and extrinsic parameters.
        :param cameraData: object structured as follows
        camera_data = {
            'intrinsic': {
                'fx': fx
                'fy': fy,
                'u0': u0,
                'v0': v0
            },
            'extrinsic': {
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll
            }
        }
        """

        # Extract params
        self.cameraData = cameraData
        self.yaw = cameraData['extrinsic']['yaw']
        self.pitch = cameraData['extrinsic']['pitch']
        self.roll = cameraData['extrinsic']['roll']
        self.x = cameraData['extrinsic']['x']
        self.y = cameraData['extrinsic']['y']
        self.z = cameraData['extrinsic']['z']
        self.fx = cameraData['intrinsic']['fx']
        self.fy = cameraData['intrinsic']['fy']
        self.u0 = cameraData['intrinsic']['u0']
        self.v0 = cameraData['intrinsic']['v0']
        self.computeCameraMatrix()

    def getIntrinsicMatrix(self):
        return self.K

    def getWorldToCameraProjection(self):
        return self.P

    def getCameraToWorldDirProjection(self):
        return np.linalg.inv(self.P[:,0:3]) # if P = (M|m), return M

    def projectImagePointToWorldDirection(self, image_point):
        image_point_h = np.float32([image_point[0], image_point[1], 1])
        world_direction = self.getCameraToWorldDirProjection().dot(image_point_h)
        return world_direction

    def projectWorldPointsToImagePoints(self, world_points): # world_points = [[x1, y1, z1],...,[xn,yn,zn]]
        world_points_h = np.hstack((world_points, np.ones((world_points.shape[0], 1)))).T
        image_points_h = self.getWorldToCameraProjection().dot(world_points_h) # @
        image_points = image_points_h[0:2,:] / image_points_h[2,:]
        image_points = image_points.T
        return image_points

    def computeCameraMatrix(self):
        # cos/sin for yaw, pitch, roll
        c_y = cos(self.yaw)
        s_y = sin(self.yaw)
        c_p = cos(self.pitch)
        s_p = sin(self.pitch)
        c_r = cos(self.roll)
        s_r = sin(self.roll)
        # from cityscape guide
        R_c_to_v = np.float32([[c_y * c_p, c_y * s_p * s_r - s_y * c_r, c_y * s_p * c_r + s_y * s_r],
                               [s_y * c_p, s_y * s_p * s_r + c_y * c_r, s_y * s_p * c_r - c_y * s_r],
                               [- s_p, c_p * s_r, c_p * c_r]])
        t_c_to_v = np.float32([self.x, self.y, self.z])[np.newaxis].T
        self.K = np.float32([[self.fx,  0,          self.u0],
                             [0,        self.fy,    self.v0],
                             [0,        0,          1]])
        M_camera_to_video = np.float32([[0, -1, 0],
                                        [0, 0, -1],
                                        [1, 0, 0]])
        C = self.K.dot(M_camera_to_video) # @
        R = R_c_to_v.T
        rpy, _ = cv2.Rodrigues(R_c_to_v)
        self.t = - R.dot(t_c_to_v)
        self.Rt = np.hstack((R, self.t))
        self.P = C.dot(self.Rt)
