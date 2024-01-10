# ROS
import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy, qos_profile_system_default
from rclpy.node import Node
from rclpy.publisher import Publisher
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
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
# Other
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import random
from ultralytics import YOLO
from ultralytics.yolo.engine.results import Results
from functools import partial
import subprocess
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
                ('publish_2d', True),
                ('publish_3d', True),
                ('exclude_border_objects', False),
                ('confidence_threshold', 0.5),
                ('iou_nms', 0.35),
                ('multi_topics', False),
                ('image_topic_list', ['topic1', 'topic2']),
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
                        'car': 2.0,
                        'motorcycle': 2.0,
                        'truck': 2.0}
        
        publish_3d = self.get_parameter('publish_3d').value
        multi_topics = self.get_parameter('multi_topics').value
        if publish_3d and multi_topics:
            self.get_logger().info("Cannot publish 3D detections with multiple topics")
            self.destroy_node()
            raise RuntimeError("Cannot publish 3D detections with multiple topics")
        
        # Keep all detections
        # qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,\
        #                         depth=10,\
        #                         reliability=QoSReliabilityPolicy.RELIABLE,\
        #                         durability=QoSDurabilityPolicy.VOLATILE,\
        #                         liveliness=QoSLivelinessPolicy.AUTOMATIC,\
        #                         deadline=qos_profile_system_default.deadline,\
        #                         lifespan=rclpy.duration.Duration(seconds=5.0),\
        #                         liveliness_lease_duration=qos_profile_system_default.liveliness_lease_duration)
        
        # Real-time
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,\
                                depth=5,\
                                reliability=qos_profile_system_default.reliability,\
                                durability=QoSDurabilityPolicy.VOLATILE,\
                                liveliness=QoSLivelinessPolicy.AUTOMATIC,\
                                deadline=qos_profile_system_default.deadline,\
                                lifespan=rclpy.duration.Duration(seconds=3.0),\
                                liveliness_lease_duration=qos_profile_system_default.liveliness_lease_duration)

        if multi_topics:
            image_topic_list = self.get_parameter('image_topic_list').value
            self.to_resize = {}
            self._sub_list = []
            self._pub_list = []
            for topic in image_topic_list:
                self.get_logger().info("Subscribe to {}".format(topic))

                parts = topic.split('/')
                # Take all parts except the last one and add det2d
                det_topic = '/'.join(parts[:-1])
                det_topic += "/det2d"
                # Resize the windows only the first time they are shown
                self.to_resize[det_topic] = True 

                # self.get_logger().info("Publish on {}".format(det_topic))

                pub2d = self.create_publisher(Detection2DArray, det_topic, qos_profile=qos_profile)
                self._pub_list.append(pub2d)

                cb_func = partial(self.detect_objects2D, publisher=pub2d)

                sub_img = self.create_subscription(Image, topic, cb_func, qos_profile=qos_profile)
                self._sub_list.append(sub_img)


        else:

            self._pub2d = self.create_publisher(Detection2DArray, 'detection_2d', qos_profile=qos_profile)

            if publish_3d:
                sub1 = Subscriber(self, Image, "to_detect")
                sub2 = Subscriber(self, CameraInfo, "camera_info")
                sub3 = Subscriber(self, Image, "depth")
                self._tss = TimeSynchronizer([sub1, sub2, sub3], queue_size=5)
                self._tss.registerCallback(self.detect_objects)

                self._pub = self.create_publisher(Detection3DArray, 'detection_3d', qos_profile=qos_profile)

                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
            else:
                cb_func = partial(self.detect_objects2D, publisher=self._pub2d)
                self._sub_img = self.create_subscription(Image, "to_detect", cb_func, qos_profile=qos_profile)

        

        # Load a pretrained YOLO model
        self.get_logger().info("Load Yolo model")
        self.yolo = YOLO('yolov8m.pt')
        self.yolo.to("cuda:0")

        self._class_to_color = {}

        # Obtain the enumeration equivalent to the classes i want to detect
        self.classes_numbers = [key for key, value in self.yolo.names.items() if value in classes_to_detect]

        self.br = CvBridge()
        self.no_image_detected_yet = True
        self.get_logger().info("Ready to detect")

    def parameter_callback(self, params):
        '''
        Function used to close the cv2 window when the show debug parameter is set to False
        '''
        for param in params:
            if param.name == 'show_debug' and param.value == False:
                cv2.destroyAllWindows()
        return SetParametersResult(successful=True)
        
    def detect_objects2D(self, data: Image, publisher: Publisher):
        """
        Callback function, it is called everytime an image is published on the given topic. It calls the yolo predict and publishes the bounding boxes 2D,
        It can also show the debug image if the parameter is set
        """
        publish_2d = self.get_parameter('publish_2d').value
        exclude_border_objects = self.get_parameter('exclude_border_objects').value

        # Display the message on the console if it is the first time
        if self.no_image_detected_yet:
            self.get_logger().info("Started detecting images")
            self.no_image_detected_yet = False

        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)

        # Perform object detection on an image using the model
        results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=float(self.get_parameter('confidence_threshold').value),
                classes=self.classes_numbers,
                iou = float(self.get_parameter('iou_nms').value)
            )
        results: Results = results[0].cpu()


        if(publish_2d):
            detections2d_msg = Detection2DArray()
            detections2d_msg.header = data.header


        for box_data in results.boxes:

            # get label and score
            label = self.yolo.names[int(box_data.cls)]
            score = float(box_data.conf)
            # get track id
            track_id = ""
            if box_data.is_track:
                track_id = str(int(box_data.id))

            # create hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = label
            hypothesis.score = score

            # get boxes values
            box = box_data.xywh[0]

            uv_coordinates = (box[0], box[1])
            bbox_image_size = (box[2], box[3])

            # bottom-left
            min_pt = (int(uv_coordinates[0] - bbox_image_size[0] / 2.0),
                            int(uv_coordinates[1] - bbox_image_size[1] / 2.0))
            # top-right
            max_pt = (int(uv_coordinates[0] + bbox_image_size[0] / 2.0),
                            int(uv_coordinates[1] + bbox_image_size[1] / 2.0))
            
            if (exclude_border_objects and (min_pt[0]<=5 or min_pt[1]<=5 or max_pt[0] >= data.width-5 or max_pt[1]>= data.height-5)):
                continue

            if(publish_2d):
                detection2d = Detection2D()
                detection2d.header = detections2d_msg.header

                detection2d.bbox.center.x = float(uv_coordinates[0])
                detection2d.bbox.center.y = float(uv_coordinates[1])
                detection2d.bbox.size_x = float(bbox_image_size[0])
                detection2d.bbox.size_y = float(bbox_image_size[1])

                detection2d.results.append(hypothesis)
                detections2d_msg.detections.append(detection2d)

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
                            0.5, color, 1, cv2.LINE_AA)

        # publish detections
        if(publish_2d):
            publisher.publish(detections2d_msg)

        if(self.get_parameter('show_debug').value):
            # Display image
            cv2.namedWindow(publisher.topic, cv2.WINDOW_NORMAL)
            cv2.imshow(publisher.topic, cv_image)
            if self.to_resize[publisher.topic]:
                def get_min_dim():
                    output = subprocess.Popen('xrandr | grep "\*" | cut -d" " -f4',shell=True, stdout=subprocess.PIPE).communicate()[0].decode('UTF-8')
                    min_w = 10000
                    min_h = 10000
                    for screen in output.split():
                        resolution = screen.split('x')
                        if min_w > int(resolution[0]):
                            min_w = int(resolution[0])
                        if min_h > int(resolution[1]):
                            min_h = int(resolution[1])
                    return min_w, min_h
                min_w, min_h = get_min_dim()

                order = 0
                for i,key in enumerate(self.to_resize.keys()):
                    if key == publisher.topic:
                        order = i
                        break
                l = 500
                v = 300
                l_pad = l + 40
                v_pad = v + 75
                tot_h_pos = min_w // l_pad
                tot_v_pos = min_h // v_pad
                h_pos = order % tot_h_pos
                v_pos = (order // tot_h_pos) % tot_v_pos
                x = 5 + h_pos * l_pad
                y = 5 + v_pos * v_pad

                cv2.moveWindow(publisher.topic, x, y)
                cv2.resizeWindow(publisher.topic, l, v)
                self.to_resize[publisher.topic] = False
            cv2.waitKey(1)



    def detect_objects(self, data: Image, camera_info: CameraInfo, depth: Image):
        """
        Callback function, it is called everytime an image is published on the given topic. It calls the yolo predict and publishes the bounding boxes,
        It can also show the debug image if the parameter is set
        """
        publish_3d = self.get_parameter('publish_3d').value
        publish_2d = self.get_parameter('publish_2d').value

        # Display the message on the console if it is the first time
        if self.no_image_detected_yet:
            self.get_logger().info("Started detecting images")
            self.no_image_detected_yet = False

        if(publish_3d):
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
    
            # Convert ROS Image message to OpenCV image
            cv_dep = self.br.imgmsg_to_cv2(depth, "32FC1")

            detections3d_msg = Detection3DArray()
            detections3d_msg.header = data.header
            camera_model = PinholeCameraModel()
            camera_model.fromCameraInfo(camera_info)

        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)

        # Perform object detection on an image using the model
        results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=float(self.get_parameter('confidence_threshold').value),
                classes=self.classes_numbers,
                iou = float(self.get_parameter('iou_nms').value)
            )
        results: Results = results[0].cpu()


        if(publish_2d):
            detections2d_msg = Detection2DArray()
            detections2d_msg.header = data.header


        for box_data in results.boxes:

            # get label and score
            label = self.yolo.names[int(box_data.cls)]
            score = float(box_data.conf)
            # get track id
            track_id = ""
            if box_data.is_track:
                track_id = str(int(box_data.id))

            # create hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = label
            hypothesis.score = score

            # get boxes values
            box = box_data.xywh[0]

            uv_coordinates = (box[0], box[1])
            bbox_image_size = (box[2], box[3])

            # bottom-left
            min_pt = (int(uv_coordinates[0] - bbox_image_size[0] / 2.0),
                            int(uv_coordinates[1] - bbox_image_size[1] / 2.0))
            # top-right
            max_pt = (int(uv_coordinates[0] + bbox_image_size[0] / 2.0),
                            int(uv_coordinates[1] + bbox_image_size[1] / 2.0))

            if(publish_2d):
                detection2d = Detection2D()
                detection2d.header = detections2d_msg.header

                detection2d.bbox.center.x = float(uv_coordinates[0])
                detection2d.bbox.center.y = float(uv_coordinates[1])
                detection2d.bbox.size_x = float(bbox_image_size[0])
                detection2d.bbox.size_y = float(bbox_image_size[1])

                detection2d.results.append(hypothesis)
                detections2d_msg.detections.append(detection2d)

            if(publish_3d):
                detection3d = Detection3D()
                detection3d.header = detections3d_msg.header

                # The depth data is stored inside the cv_dep numpy array -> y are the rows and x are columns
                z_value = cv_dep[int(uv_coordinates[1])][int(uv_coordinates[0])]

                # Project the 3 points
                detection3d.bbox.center.position.x, \
                    detection3d.bbox.center.position.y, \
                        detection3d.bbox.center.position.z  = project_to_3D_space(camera_model, uv_coordinates, z_value)
                
                min_pt_3d = project_to_3D_space(camera_model, min_pt, z_value)
                max_pt_3d = project_to_3D_space(camera_model, max_pt, z_value)
                
                # Add an orientation to remove the tilt of the camera and place the bbox horizontally
                detection3d.bbox.center.orientation = q_to_apply

                # Compute the size of the bounding box looking at the projections of the two vertexes
                detection3d.bbox.size.x = float(max_pt_3d[0]-min_pt_3d[0])
                detection3d.bbox.size.y = float(max_pt_3d[1]-min_pt_3d[1])
                detection3d.bbox.size.z = self.class_to_z_val[label]

                detection3d.results.append(hypothesis)
                # append msg
                detections3d_msg.detections.append(detection3d)


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
                            0.5, color, 1, cv2.LINE_AA)

        # publish detections
        if(publish_3d):
            self._pub.publish(detections3d_msg)
        if(publish_2d):
            self._pub2d.publish(detections2d_msg)

        if(self.get_parameter('show_debug').value):
            # Display image
            cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
            cv2.imshow("Detection", cv_image)
            cv2.waitKey(1)
        # Publish debug image to a topic
        # self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding=data.encoding))
        

def main(args=None):
    rclpy.init(args=args)

    yolo_detector = YoloDetector()

    # rclpy.spin(yolo_detector)
    
    try:
        rclpy.spin(yolo_detector)
    except:
        print("Yolo Detector Terminated")


    # Destroy the node explicitly
    yolo_detector.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()