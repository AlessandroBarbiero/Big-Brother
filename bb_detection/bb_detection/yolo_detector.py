import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image # Image is the message type
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import random
from ultralytics import YOLO
from ultralytics.yolo.engine.results import Results

from rcl_interfaces.msg import SetParametersResult

class YoloDetector(Node):

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

        self._pub = self.create_publisher(Detection2DArray, 'topic', 10)
        self._sub = self.create_subscription(Image, 'to_detect', self.listener_callback, 10)
        self._sub # prevent unused variable warning

        # Load a pretrained YOLO model
        self.yolo = YOLO('yolov8m.pt')
        self.yolo.to("cuda:0")
        self._class_to_color = {}

        self.br = CvBridge()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'show_debug' and param.value == False:
                cv2.destroyAllWindows()
        return SetParametersResult(successful=True)
    
    def listener_callback(self, data: Image):
        """
        Callback function, it is called everytime an image is published on the given topic. It calls the yolo predict and publishes the bounding boxes,
        It can also show the debug image if the parameter is set
        """

        # Display the message on the console
        self.get_logger().info("Received image")
    
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)

        # Perform object detection on an image using the model
        results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=float(self.get_parameter('confidence_threshold').value),
            )
        results: Results = results[0].cpu()

        # create detections msg
        detections_msg = Detection2DArray()
        detections_msg.header = data.header

        for box_data in results.boxes:

            detection = Detection2D()

            # get label and score
            label = self.yolo.names[int(box_data.cls)]
            score = float(box_data.conf)

            # get boxes values
            box = box_data.xywh[0]
            detection.bbox.center.x = float(box[0])
            detection.bbox.center.y = float(box[1])
            detection.bbox.size_x = float(box[2])
            detection.bbox.size_y = float(box[3])

            # get track id
            track_id = ""
            if box_data.is_track:
                track_id = str(int(box_data.id))
            # detection.id = track_id

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

                min_pt = (round(detection.bbox.center.x - detection.bbox.size_x / 2.0),
                            round(detection.bbox.center.y - detection.bbox.size_y / 2.0))
                max_pt = (round(detection.bbox.center.x + detection.bbox.size_x / 2.0),
                            round(detection.bbox.center.y + detection.bbox.size_y / 2.0))
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

    rclpy.spin(yolo_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()