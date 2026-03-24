from ultralytics import YOLO
import cv2
import cvzone
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        # Webcam
        # self.cap = cv2.VideoCapture(0)  # Modify this how you want to use whatever camera is desired
        # self.cap.set(3, 1280)
        # self.cap.set(4, 720)

        # Video
        self.cap = cv2.VideoCapture("/home/orca4/colcon_ws/src/orca4/vision_testing/vision_pkg/card_detection_test_video.mp4")
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video file")

        # self.model = YOLO('./Yolo-Weights/playingCards.pt')
        self.model = YOLO('/home/orca4/colcon_ws/src/orca4/vision_testing/vision_pkg/vision_pkg/Yolo-Weights/playingCards.pt')

        self.classNames = ['10C', '10D', '10H', '10S',
                    '2C', '2D', '2H', '2S',
                    '3C', '3D', '3H', '3S',
                    '4C', '4D', '4H', '4S',
                    '5C', '5D', '5H', '5S',
                    '6C', '6D', '6H', '6S',
                    '7C', '7D', '7H', '7S',
                    '8C', '8D', '8H', '8S',
                    '9C', '9D', '9H', '9S',
                    'AC', 'AD', 'AH', 'AS',
                    'JC', 'JD', 'JH', 'JS',
                    'KC', 'KD', 'KH', 'KS',
                    'QC', 'QD', 'QH', 'QS']
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/vision/image', 10) # create a publisher for images for viewing
        self.detections_pub = self.create_publisher(Detection2DArray, '/vision/detections', 10) # create a publisher for detection data
    def timer_callback(self):
    
        # key = cv2.waitKey(1) & 0xFF
        success, img = self.cap.read()
        # if not success:
        #     # print("failed to read")
        #     self.get_logger().error("Failed to read video file")
        #     return
        if not success:
            self.get_logger().warn("End of video reached, restarting")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        results = self.model(img, stream=True)

        detection_array_msg = Detection2DArray()
        detection_array_msg.header.stamp = self.get_clock().now().to_msg()
        detection_array_msg.header.frame_id = "camera_frame"

        detected_objects = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist() # the tolist helps avoid weird issues with types
                x1_i, y1_i, x2_i, y2_i = int(x1), int(y1), int(x2), int(y2)
                # cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 3)
                w, h = x2 - x1, y2 - y1
                cvzone.cornerRect(img, (x1_i, y1_i, int(w), int(h)))
                # Confidence
                conf = math.ceil((box.conf[0] * 100)) / 100
                # Class Name
                cls = int(box.cls[0])
                if conf > 0.3:
                    detection_msg = Detection2D() # contains the data for 1 object

                    detection_msg.header = detection_array_msg.header # match the detection array

                    # Store the data for the bounding box
                    detection_msg.bbox.center.position.x = (x1 + x2) / 2.0 # Find center point x and y
                    detection_msg.bbox.center.position.y = (y1 + y2) / 2.0
                    detection_msg.bbox.size_x = w
                    detection_msg.bbox.size_y = h # may need to make these floats

                    # Create a hypothesis which has data on what object we believe we have detected and the confidence
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = self.classNames[cls]
                    hypothesis.hypothesis.score = float(conf)

                    detection_msg.results.append(hypothesis)
                    detection_array_msg.detections.append(detection_msg)

                    detected_objects.append(self.classNames[cls])
                cvzone.putTextRect(img, f' {self.classNames[cls]} {conf}', (max(0, x1_i), max(30, y1_i)), scale=0.8,
                                thickness=1) # this is just to help visualize
        # Get rid of repeated detections of the same card
        detected_objects = list(set(detected_objects))
        self.get_logger().info(f"Detected objects: {detected_objects}")
        # print(f"working : {detected_objects} ")
        # if len(detected_objects) != 0: # For debugging and testing
        #     print(f"detected : {detected_objects} ")
        # This handles the display state for inputting hole cards
        cvzone.putTextRect(img, f'Press c to do something',
                                (50, 160), 1.5, 2,(255, 255, 255), (200, 0, 200))
        # if key == ord('c'):  # If c is pressed, do something # removing because it references key
        #     pass
        # if key == ord('a'):  # If a is pressed do something
        #     pass
        cvzone.putTextRect(img, f'Detected: {detected_objects}', (50, 50), 1, 2)
        # cv2.imshow("Image", img) # disabling for now because it's breaking in WSL because GUI

        # cv2.waitKey(1)
        # if key == ord('q'):
        #     break

        
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8') # convert form opencv style images to ros style formatting
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "camera_frame"
        self.image_pub.publish(ros_img) # publish the image as a topic

        



        self.detections_pub.publish(detection_array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()