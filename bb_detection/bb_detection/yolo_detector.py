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

class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')
        self._pub = self.create_publisher(Detection2DArray, 'topic', 10)

        # Create the subscriber. This subscriber will receive an Image
        # from the to_detect topic. The queue size is 10 messages.
        self._sub = self.create_subscription(Image, 'to_detect', self.listener_callback, 10)
        self._sub # prevent unused variable warning

        # Load a pretrained YOLO model
        self.yolo = YOLO('yolov8m.pt')
        # self.yolo.fuse()
        self.yolo.to("cuda:0")
        self._class_to_color = {}

        self.br = CvBridge()
        self.threshold = 0.5

    def listener_callback(self, data: Image):
        """
        Callback function, it is called everytime an image is published on the given topic.
        """
    
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data)

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)

        # Perform object detection on an image using the model
        results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
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

        # publish detections and dbg image
        self._pub.publish(detections_msg)
        # self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding=data.encoding))

        # Display the message on the console
        self.get_logger().info("Received image")

        # Display image
        cv2.imshow("Detection", cv_image)
        
        cv2.waitKey(1)


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