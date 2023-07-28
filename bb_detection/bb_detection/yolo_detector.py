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
import random
from ultralytics import YOLO
from ultralytics.yolo.engine.results import Results
# My scripts
from .utility import *


class YoloDetector(Node):
    '''
    ROS2 Node that performs Yolo detection and classification on the images published on the topic 'to_detect'.\n
    Exploiting the data published on the topics 'camera_info' and 'depth' it produces a list of 3D bounding boxes
    and publish them on the topic 'detection_3d'.\n
    The node can detect and classify objects of the classes: 'person', 'bicycle', 'car', 'motorcycle' and 'truck'.\n
    The depth of the bounding box is obtained heuristically based on the type of the recognized class.\n
    ROS2 parameters:
        'show_debug': Allow to display a debug image showing the 2D detections on the given image (Default: True),
        'confidence_threshold': Limit threshold over which the node consider a valid detection (Default: 0.5)
    '''

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('show_debug', True),
                ('confidence_threshold', 0.5)
            ]
        )
        self.add_on_set_parameters_callback(self.parameter_callback)

        classes_to_detect = ['person',
                             'bicycle',
                             'car',
                             'motorcycle',
                             'truck']
        
        self.class_to_z_val = {'person': 1.0,
                        'bicycle': 1.5,
                        'car': 2.5,
                        'motorcycle': 2.0,
                        'truck': 3.0}
        

        sub1 = Subscriber(self, Image, "to_detect")
        sub2 = Subscriber(self, CameraInfo, "camera_info")
        sub3 = Subscriber(self, Image, "depth")
        self._tss = TimeSynchronizer([sub1, sub2, sub3], queue_size=5)
        self._tss.registerCallback(self.detect_3d)

        self._pub = self.create_publisher(Detection3DArray, 'detection_3d', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load a pretrained YOLO model
        self.yolo = YOLO('yolov8m.pt')
        self.yolo.to("cuda:0")

        self._class_to_color = {}

        # Obtain the enumeration equivalent to the classes i want to detect
        self.classes_numbers = [key for key, value in self.yolo.names.items() if value in classes_to_detect]

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
    
    
    def detect_3d(self, data: Image, camera_info: CameraInfo, depth: Image):
        """
        Callback function, it is called everytime an image is published on the given topic. It calls the yolo predict and publishes the bounding boxes,
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

        # Display the message on the console if it is the first time
        if self.no_image_detected_yet:
            self.get_logger().info("Started detecting images")
            self.no_image_detected_yet = False
    
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)
        cv_dep = self.br.imgmsg_to_cv2(depth, "32FC1")

        # Perform object detection on an image using the model
        results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=float(self.get_parameter('confidence_threshold').value),
                classes=self.classes_numbers
            )
        results: Results = results[0].cpu()

        # create detections msg
        detections_msg = Detection3DArray()
        detections_msg.header = data.header

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)


        for box_data in results.boxes:

            detection = Detection3D()

            # get label and score
            label = self.yolo.names[int(box_data.cls)]
            score = float(box_data.conf)

            # get boxes values
            box = box_data.xywh[0]

            uv_coordinates = (box[0], box[1])
            bbox_image_size = (box[2], box[3])
            # The depth data is stored inside the cv_dep numpy array -> y are the rows and x are columns
            z_value = cv_dep[int(uv_coordinates[1])][int(uv_coordinates[0])]

            # bottom-left
            min_pt = (int(uv_coordinates[0] - bbox_image_size[0] / 2.0),
                            int(uv_coordinates[1] - bbox_image_size[1] / 2.0))
            # top-right
            max_pt = (int(uv_coordinates[0] + bbox_image_size[0] / 2.0),
                            int(uv_coordinates[1] + bbox_image_size[1] / 2.0))

            # Project the 3 points
            detection.bbox.center.position.x, \
                detection.bbox.center.position.y, \
                    detection.bbox.center.position.z  = project_to_3D_space(camera_model, uv_coordinates, z_value)
            
            min_pt_3d = project_to_3D_space(camera_model, min_pt, z_value)
            max_pt_3d = project_to_3D_space(camera_model, max_pt, z_value)
            
            # Add an orientation to remove the tilt of the camera and place the bbox orizontally
            detection.bbox.center.orientation = q_to_apply

            # Compute the size of the bounding box looking at the projections of the two vertexes
            detection.bbox.size.x = float(max_pt_3d[0]-min_pt_3d[0])
            detection.bbox.size.y = float(max_pt_3d[1]-min_pt_3d[1])
            detection.bbox.size.z = self.class_to_z_val[label]

            # get track id
            track_id = ""
            if box_data.is_track:
                track_id = str(int(box_data.id))

            # create hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = label
            hypothesis.score = score
            detection.results.append(hypothesis)

            if(self.get_parameter('show_debug').value):
                # draw boxes for debug
                if label not in self._class_to_color:
                    r = random.randint(0, 255)
                    g = random.randint(0, 255)
                    box_data = random.randint(0, 255)
                    self._class_to_color[label] = (r, g, box_data)
                color = self._class_to_color[label]
             
                cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

                label = "{} ({}) ({:.3f})".format(label, str(track_id), score)
                pos = (min_pt[0] + 5, min_pt[1] + 25)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, label, pos, font,
                            1, color, 1, cv2.LINE_AA)

            # append msg
            detections_msg.detections.append(detection)

        # publish detections
        self._pub.publish(detections_msg)

        if(self.get_parameter('show_debug').value):
            # Display image
            cv2.imshow("Detection", cv_image)
            cv2.waitKey(1)
        # Publish debug image to a topic
        # self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding=data.encoding))
        

def main(args=None):
    rclpy.init(args=args)

    yolo_detector = YoloDetector()

    try:
        rclpy.spin(yolo_detector)
    except:
        print("Yolo Detector Terminated")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()