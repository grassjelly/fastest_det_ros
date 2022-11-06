#!/usr/bin/env python
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, CameraInfo, Image
from std_msgs.msg import Float32MultiArray, Header, ColorRGBA
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import cv2
from cv_bridge import CvBridge, CvBridgeError
from yolo_fastest_ros.inference import YoloFastest


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self._camera_topic = self.get_parameter('camera_topic').value
        
        self._cv_bridge = CvBridge()

        self._image = None
        self._bbox_msg = None

        self._image_subscriber = self.create_subscription(
            Image,
            self._camera_topic,
            self._camera_callback,
            qos_profile_sensor_data,
        )
        self._image_subscriber
        
        self._bbox_publisher = self.create_publisher(
            Detection2DArray, 
            '/objects', 
            10
        ) 

        inference_timer = self.create_timer(0.2, self._inference_callback)

        self._detector = YoloFastest()

        self.get_logger().info("Object detector ready!")

    def _inference_callback(self):
        if self._image is None:
            return

        start = time.perf_counter()
     

        self._detector.predict(self._image)
        end = time.perf_counter()
        t = (end - start) * 1000.
        self.get_logger().info(f"forward time: {t}")

    def _camera_callback(self, image_msg):
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        self._image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

    def _create_bbox(self, x, y, id, label):
        pass


def main(args=None):
    rclpy.init(args=args)

    o = ObjectDetector()
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(o)
    executor.spin()

if __name__ == '__main__':
    main()