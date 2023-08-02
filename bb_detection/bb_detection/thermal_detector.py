# ROS
import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber
from rcl_interfaces.msg import SetParametersResult
from image_geometry import PinholeCameraModel
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
# ROS messages
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection3D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import TransformStamped
# Other
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
# My files
from .utility import *
from .bev.Bev import Bev
from .bev.Camera import Camera

SENSORS_FRAME = "sensors_home/sensors_frame"

class ThermalDetector(Node):
    '''
    ROS2 Node that performs detection and classification on the images published on the topic 'to_detect'. 
    The node is made to work with the images produced by the semantic segmentation camera of the Carla simulator 
    but it can be applyed on whatever image that displays segmented objects with a mask, each class of object has its own color\n
    Exploiting the data published on the topic 'camera_info' it produces a list of 3D bounding boxes
    and publish them on the topic 'detection_3d'.\n
    The node can detect and classify objects of the classes: 'person' and 'vehicle'.\n
    The depth and height of the bounding box is obtained heuristically based on the type of the recognized class.\n
    ROS2 parameters:
        'show_debug': Allow to display a debug image showing the 2D detections on the given image (Default: True),
    '''

    def __init__(self):
        super().__init__('thermal_detector')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('show_debug', True)
            ]
        )
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.class_to_depth = {'person': 1.0,
                            'vehicle': 2.0
                            }
        self.class_to_height = {'person': 1.8,
                            'vehicle': 1.6
                            }
        
        self.colors = {'person': (60, 20, 220),
                        'vehicle': (142, 0, 0)
                        }

        sub1 = Subscriber(self, Image, "to_detect")
        sub2 = Subscriber(self, CameraInfo, "camera_info")
        self._tss = TimeSynchronizer([sub1, sub2], queue_size=5)
        self._tss.registerCallback(self.detect_3d)

        self._pub = self.create_publisher(Detection3DArray, 'detection_3d', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.class_to_bbox_color = {'person': (255, 0, 0),
                                'vehicle': (10, 255, 0)
                                }

        self.br = CvBridge()
        self.no_image_detected_yet = True

    def parameter_callback(self, params):
        '''
        Function used to close the cv2 window when the show debug parameter is set to False
        '''
        for param in params:
            if param.name == 'show_debug' and param.value == False:
                cv2.destroyAllWindows()
        return SetParametersResult(successful=True)
    
    
    def detect_3d(self, data: Image, camera_info: CameraInfo):
        """
        Callback function, it is called everytime an image is published on the given topic. It transform the semantic segmentation into 
        bounding boxes and publishes the 3D bounding boxes,
        It can also show the debug image if the parameter is set
        """
        
        # Compute the transformation from the Default frame to the sensor frame to compute the bev projection
        from_frame_rel = SENSORS_FRAME
        to_frame_rel = DEFAULT_FRAME

        try:
            tf : TransformStamped = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        roll, pitch, yaw = euler_from_quaternion(tf.transform.rotation)

        # Display the message on the console if it is the first time
        if self.no_image_detected_yet:
            self.get_logger().info("Started detecting images")
            self.no_image_detected_yet = False
    
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)


        # Prepare Bird Eye View projection
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)
        cameraData = {
            'intrinsic': {
                'fx': camera_model.fx(),
                'fy': camera_model.fy(),
                'u0': camera_model.cx(),
                'v0': camera_model.cy()
            },
            'extrinsic': {
                'x': tf.transform.translation.x,
                'y': tf.transform.translation.y,
                'z': tf.transform.translation.z,
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll
            }
        }
        camera = Camera(cameraData)
        bev_out_view = [0, 10, -20, 20]
        bev_out_image_width = 1000
        bev_out_image_size = [np.nan, bev_out_image_width] 
        bev_obj = Bev(camera, bev_out_view, bev_out_image_size)


        # Create detections msg
        detections_msg = Detection3DArray()
        detections_msg.header = data.header
        detections_msg.header.frame_id = DEFAULT_FRAME

        # Create the bbox starting from semseg
        for class_name, target_color in self.colors.items():
            # Find all pixels with the target_color in the image
            target_pixels = np.all(cv_image == target_color, axis=-1)

            # Find contours of the target pixels
            contours, _ = cv2.findContours(target_pixels.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # For debug window
            bounding_box_color = self.class_to_bbox_color[class_name]

            # Draw bounding boxes around the detected regions
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                left_base_pt = (x, y)
                right_base_pt = (x+w, y)

                detection = Detection3D()
                detection.header = detections_msg.header

                # Project base points into Bird Eye View Projection
                points = np.array([left_base_pt, right_base_pt])
                bev_points = bev_obj.projectImagePointsToBevPoints(points)
                real_points = bev_obj.projectBevPointsToWorldGroundPlane(bev_points)
                real_left = (real_points[0][0], real_points[0][1])
                real_right = (real_points[1][0], real_points[1][1])
    
                detection.bbox.center.position.x = real_left[0] + (real_right[0]-real_left[0])/2
                detection.bbox.center.position.y = real_left[1] + (real_right[1]-real_left[1])/2
                detection.bbox.center.position.z = self.class_to_height[class_name]/2

                # Compute the size of the bounding box based on the class
                detection.bbox.size.x = self.class_to_depth[class_name]
                detection.bbox.size.y = float(real_right[1]-real_left[1])
                detection.bbox.size.z = self.class_to_height[class_name]

                # Create hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = class_name
                hypothesis.score = 1.0
                detection.results.append(hypothesis)

                # Append msg
                detections_msg.detections.append(detection)

                # Draw rectangles in debug image
                if(self.get_parameter('show_debug').value):
                    min_pt = left_base_pt
                    max_pt = (x+w, y+h)
                    cv2.rectangle(cv_image, min_pt, max_pt, bounding_box_color, 2)
                    pos = (min_pt[0] + 5, min_pt[1] + 25)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(cv_image, class_name, pos, font,
                                1, bounding_box_color, 1, cv2.LINE_AA)

        # Publish detections
        self._pub.publish(detections_msg)

        if(self.get_parameter('show_debug').value):
            # Display image
            cv2.imshow("Detection", cv_image)
            cv2.waitKey(1)
             

def main(args=None):
    rclpy.init(args=args)

    thermal_detector = ThermalDetector()

    try:
        rclpy.spin(thermal_detector)
    except:
        print("Thermal Detector Terminated")

    # rclpy.spin(thermal_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thermal_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()