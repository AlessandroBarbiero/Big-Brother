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
from geometry_msgs.msg import Quaternion
# Other
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
# My files
from .utility import *

class ThermalDetector(Node):

    def __init__(self):
        super().__init__('thermal_detector')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('show_debug', True)
            ]
        )
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.class_to_z_val = {'person': 1.0,
                            'vehicle': 2.0
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
        
        # Compute the transformation from the Default frame to remove tilted orientation from bboxes
        from_frame_rel = camera_info.header.frame_id
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
        
        inverse_tf_rotation = tf.transform.rotation
        inverse_tf_rotation.w = -inverse_tf_rotation.w
        r90_zaxis = Quaternion(w=0.707, x=0.0, y=0.0, z=0.707)
        q_to_apply = quaternion_multiply(inverse_tf_rotation,r90_zaxis)
        # TODO: Bird eye view projection
        # compute the angle theta between the z axis and the ground
        v = math.sqrt(tf.transform.rotation.x**2 + tf.transform.rotation.y**2 + tf.transform.rotation.z**2)
        theta = 2 * math.atan2(v, tf.transform.rotation.w)
        # I have to use this angle to get the real distance z of the object


        # Display the message on the console if it is the first time
        if self.no_image_detected_yet:
            self.get_logger().info("Started detecting images")
            self.no_image_detected_yet = False
    
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)

        # create detections msg
        detections_msg = Detection3DArray()
        detections_msg.header = data.header

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)


        # Create the bbox starting from semseg
        for class_name, target_color in self.colors.items():
            # Find all pixels with the target_color in the image
            target_pixels = np.all(cv_image == target_color, axis=-1)

            # Find contours of the target pixels
            contours, _ = cv2.findContours(target_pixels.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # draw boxes for debug
            bounding_box_color = self.class_to_bbox_color[class_name]

            # Draw bounding boxes around the detected regions
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                min_pt = (x, y)
                max_pt = (x + w, y + h)

                detection = Detection3D()

                center_coordinates = (min_pt[0] + (max_pt[0]-min_pt[0])/2, min_pt[1] + (max_pt[1]-min_pt[1])/2)

                # The depth data is stored inside the cv_dep numpy array -> y are the rows and x are columns
                # z_value = cv_dep[int(uv_coordinates[1])][int(uv_coordinates[0])]
                z_value = 1 # TODO: find the z value using trigonometry knowing the angle that the camera has with the ground

                # Project the 3 points
                detection.bbox.center.position.x, \
                    detection.bbox.center.position.y, \
                        detection.bbox.center.position.z  = project_to_3D_space(camera_model, center_coordinates, z_value)
                
                min_pt_3d = project_to_3D_space(camera_model, min_pt, z_value)
                max_pt_3d = project_to_3D_space(camera_model, max_pt, z_value)
                
                # Add an orientation to remove the tilt of the camera and place the bbox orizontally
                detection.bbox.center.orientation = q_to_apply

                # Compute the size of the bounding box looking at the projections of the two vertexes
                detection.bbox.size.x = float(max_pt_3d[0]-min_pt_3d[0])
                detection.bbox.size.y = float(max_pt_3d[1]-min_pt_3d[1])
                detection.bbox.size.z = self.class_to_z_val[class_name]

                # create hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = class_name
                hypothesis.score = 1.0
                detection.results.append(hypothesis)

                # append msg
                detections_msg.detections.append(detection)

                if(self.get_parameter('show_debug').value):
                    cv2.rectangle(cv_image, min_pt, max_pt, bounding_box_color, 2)
                    pos = (min_pt[0] + 5, min_pt[1] + 25)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(cv_image, class_name, pos, font,
                                1, bounding_box_color, 1, cv2.LINE_AA)

        # publish detections
        self._pub.publish(detections_msg)

        if(self.get_parameter('show_debug').value):
            # Display image
            cv2.imshow("Detection", cv_image)
            cv2.waitKey(1)
             

def main(args=None):
    rclpy.init(args=args)

    thermal_detector = ThermalDetector()

    # try:
    #     rclpy.spin(thermal_detector)
    # except:
    #     print("Thermal Detector Terminated")

    rclpy.spin(thermal_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thermal_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()