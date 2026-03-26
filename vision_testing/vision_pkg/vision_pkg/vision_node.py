from ultralytics import YOLO
import cv2
import cvzone
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

# from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
# from geometry_msgs.msg import Pose2D
from robosub_msgs.msg import Detection3D, DetectionArray


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        # Webcam
        # self.cap = cv2.VideoCapture(0)  # Modify this how you want to use whatever camera is desired
        # self.cap.set(3, 1280)
        # self.cap.set(4, 720)

        # Video
        # self.cap = cv2.VideoCapture("/home/orca4/colcon_ws/src/orca4/vision_testing/vision_pkg/card_detection_test_video.mp4")
        # if not self.cap.isOpened():
        #     self.get_logger().error("Failed to open video file")

        # Subscribe to ZED left image and depth image, synchronized
        self.image_sub = Subscriber(self, Image, '/zed/zed_node/left/image_rect_color')
        self.depth_sub = Subscriber(self, Image, '/zed/zed_node/depth/depth_registered')


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
        # self.timer = self.create_timer(0.05, self.timer_callback)
        self.bridge = CvBridge()

        # Sync the two topics — they should arrive together
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            queue_size=20,
            slop=0.2  # 200ms tolerance
        )
        self.sync.registerCallback(self.image_callback)

        self.image_pub = self.create_publisher(Image, '/vision/image', 10) # create a publisher for images for viewing
        self.detections_pub = self.create_publisher(DetectionArray, '/vision/detections', 10) # create a publisher for detection data
    def image_callback(self, img_msg, depth_msg):

        # now = self.get_clock().now().nanoseconds # add this back later
        # if now - self.last_inference_time < 0.2:  # 5 Hz
        #     return
        # self.last_inference_time = now

        # self.get_logger().info( # debugging
        #     f"Image ts: {img_msg.header.stamp.sec}.{img_msg.header.stamp.nanosec} | "
        #     f"Depth ts: {depth_msg.header.stamp.sec}.{depth_msg.header.stamp.nanosec}"
        # )
        if img_msg is None or depth_msg is None:
            self.get_logger().warn("Received None messages")
            return

        if len(img_msg.data) == 0 or len(depth_msg.data) == 0:
            self.get_logger().warn("Empty image or depth data")
            return

        # key = cv2.waitKey(1) & 0xFF
        # success, img = self.cap.read()
        # if not success:
        #     # print("failed to read")
        #     self.get_logger().error("Failed to read video file")
        #     return
        # if not success:
        #     self.get_logger().warn("End of video reached, restarting")
        #     self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        #     return

        # Convert ROS images to OpenCV
        try:
            # img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
            if img is None:
                self.get_logger().error("Image is None")
                return

            # Convert BGRA → BGR
            if len(img.shape) == 3 and img.shape[2] == 4:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed (rgb): {e}")
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if depth is None:
                self.get_logger().error("Depth img is None")
                return

            # Convert BGRA → BGR
            if len(depth.shape) == 3 and depth.shape[2] == 4:
                depth = cv2.cvtColor(depth, cv2.COLOR_BGRA2BGR)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed (depth): {e}")
            return
        # img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        # depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        # depth is now a float32 array where each value is meters

        results = self.model(img, stream=True)

        detection_array_msg = DetectionArray()
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
                    detection_msg = Detection3D() # contains the data for 1 object

                    detection_msg.header = detection_array_msg.header # match the detection array

                    # Store the data for the bounding box
                    cen_x = int((x1 + x2) // 2)
                    cen_y = int((y1 + y2) // 2)
                    # detection_msg.bbox.center.position.x = cen_x # Find center point x and y
                    # detection_msg.bbox.center.position.y = cen_y
                    # detection_msg.bbox.size_x = w
                    # detection_msg.bbox.size_y = h # may need to make these floats

                    detection_msg.bbox_x = float(cen_x)
                    detection_msg.bbox_y = float(cen_y)
                    detection_msg.bbox_width = float(w)
                    detection_msg.bbox_height = float(h)

                    # # Create a hypothesis which has data on what object we believe we have detected and the confidence
                    # hypothesis = ObjectHypothesisWithPose()
                    # hypothesis.hypothesis.class_id = self.classNames[cls]
                    # hypothesis.hypothesis.score = float(conf)

                    detection_msg.class_id = self.classNames[cls] #record the name of what we detect
                    detection_msg.confidence = float(conf)


                    # detection_msg.results.append(hypothesis)

                    # Now get depth information

                    # The depth image is a 2D array matching the color image resolution
                    # depth[row, col] = distance in meters at that pixel
                    # Note: row = y, col = x (numpy array indexing)
                    # object_depth = depth[cen_y, cen_x]


                    # ZED can't always compute depth — glass, shiny surfaces, too far away, etc.
                    # Those pixels are set to inf or nan instead of a real number
                    # np.isfinite() returns False for both inf and nan, filtering them out

                    patch = depth[cen_y-5:cen_y+5, cen_x-5:cen_x+5]
                    valid = patch[np.isfinite(patch)]
                    if len(valid) > 0:
                        detection_msg.depth = float(np.median(valid))
                        detection_msg.depth_valid = True
                    else:
                        detection_msg.depth = -1.0
                        detection_msg.depth_valid = False

                    detection_array_msg.detections.append(detection_msg)

                    # if np.isfinite(object_depth):
                    #     detection_msg.depth = object_depth # does this work?
                    #     # detected_objects.append({
                    #     #     'depth_m': round(float(object_depth), 2),  # distance to object in meters
                    #     # })
                    # else:
                    #     detection_msg.depth = -1.0 # does this work?

                    # detection_array_msg.detections.append(detection_msg)

                    detected_objects.append(self.classNames[cls])

                    cvzone.putTextRect(img, f' {self.classNames[cls]} {conf}', (max(0, x1_i), max(30, y1_i)), scale=0.8,
                                thickness=1) # this is just to help visualize
        # Get rid of repeated detections of the same card
        detected_objects = list(set(detected_objects))
        self.get_logger().info(f"Detected objects: {detected_objects}")
        # print(f"working : {detected_objects} ")
        # if len(detected_objects) != 0: # For debugging and testing
        #     print(f"detected : {detected_objects} ")
        # cvzone.putTextRect(img, f'Press c to do something',
        #                         (50, 160), 1.5, 2,(255, 255, 255), (200, 0, 200))
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
    # node.cap.release()
    # cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()