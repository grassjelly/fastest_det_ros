#!/usr/bin/env python
import time
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import cv2
from cv_bridge import CvBridge, CvBridgeError
from fastest_det_ros.inference import FastestDet


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('camera_topic', '/camera/image')
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

        self._debug_image_publisher = self.create_publisher(
            Image, 
            '/objects_debug', 
            qos_profile_sensor_data
        ) 

        inference_timer = self.create_timer(0.2, self._inference_callback)

        self._detector = FastestDet()

        self.get_logger().info("Object detector ready!")

    def _inference_callback(self):
        if self._image is None:
            return
     
        classes, bbox, conf = self._detector.predict(self._image)
        self._publish_debug(
            self._image,
            classes,
            bbox,
            conf
        )

        obj_array = Detection2DArray()
        obj_array.detections =[]
        obj_array.header.stamp = self.get_clock().now().to_msg()

        for i, box in enumerate(bbox):
            bbox_msg = self._create_bbox_msg(box, classes[i], conf[i])
            obj_array.detections.append(bbox_msg)

        self._bbox_publisher.publish(obj_array)

    def _publish_debug(self, image, classes, bbox, conf):
        image = copy.deepcopy(self._image)
        img_cv = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        for i, box in enumerate(bbox):
            cv2.rectangle(img_cv, box[0], box[1], (200, 200, 0), 5)
            cv2.putText(img_cv, '%.2f' % conf[1], (box[0][0], box[0][1] - 5), 0, 0.7, (0, 255, 0), 2)	
            cv2.putText(img_cv, classes[i], (box[0][0], box[0][1] - 25), 0, 0.7, (0, 255, 0), 2)

        image_out = Image()
        try:
            image_out = self._cv_bridge.cv2_to_imgmsg(img_cv, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self._debug_image_publisher.publish(image_out)

    def _camera_callback(self, image_msg):
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        self._image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

    def _create_bbox_msg(self, box, label, conf):
        obj = Detection2D()
        obj_hypothesis = ObjectHypothesisWithPose()

        obj.header.stamp = self.get_clock().now().to_msg()
        obj_hypothesis.hypothesis.class_id = str(label)
        obj_hypothesis.hypothesis.score = conf
        obj.results.append(obj_hypothesis)

        obj.bbox.size_x = float(box[1][0] - box[0][0])
        obj.bbox.size_y = float(box[1][1] - box[0][1])
        obj.bbox.center.x = box[0][0] + (obj.bbox.size_x / 2.)
        obj.bbox.center.y = box[0][1] + (obj.bbox.size_y / 2.)

        return obj


def main(args=None):
    rclpy.init(args=args)

    o = ObjectDetector()
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(o)
    executor.spin()


if __name__ == '__main__':
    main()