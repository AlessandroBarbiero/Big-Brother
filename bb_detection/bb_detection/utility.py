
from geometry_msgs.msg import Quaternion
import math
from image_geometry import PinholeCameraModel

DEFAULT_FRAME = 'sensors_home'

classes_to_detect = ['person',
                     'pedestrian',
                     'bicycle',
                     'car',
                     'motorcycle',
                     'truck',
                     'vehicle'
                    ]

def quaternion_multiply(q1, q2):
    q_result = Quaternion()
    q_result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    q_result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    q_result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
    q_result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    return q_result

def euler_from_quaternion(q : Quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw # in radians

def find_3d_point_on_line(line_vector, z_value):
    '''
    Compute the position in the 3D world of a point along a line described by the line_vector 3d (x,y,z) passed
    '''
    # Normalize the line vector
    magnitude = math.sqrt(line_vector[0] ** 2 + line_vector[1] ** 2 + line_vector[2] ** 2)
    x_unit = line_vector[0] / magnitude
    y_unit = line_vector[1] / magnitude
    z_unit = line_vector[2] / magnitude
    # Calculate proportions
    a = z_value / z_unit

    # Calculate the final coordinates (x, y, z)
    x = a * x_unit
    y = a * y_unit

    return float(x), float(y), float(z_value)

def project_to_3D_space(camera_model: PinholeCameraModel, uv_coordinates: tuple, z_value: float):
    '''
    Project a 2D point of an image into a 3D space knowing the depth of the point
    :param camera_model: The cameraModel produced feeding the cameraInfo message
    :param uv_coordinates: The coordinates of the point in the image you want to project
    :param z_value: The depth of the point in the 3D space
    :returns the 3 values for x,y,z
    '''
    ray3d = camera_model.projectPixelTo3dRay(uv_coordinates)
    return find_3d_point_on_line(ray3d, z_value)
