#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2

def kelvin_to_celsius(k): return k - 273.15

class ThermalClassifier(Node):
    def __init__(self):
        super().__init__('thermal_classifier')
        self.bridge = CvBridge()
        self.declare_parameter('image_topic', '/camera/thermal/image')
        self.declare_parameter('human_temp_c', 35.0)
        self.declare_parameter('deer_temp_c', 28.0)
        self.declare_parameter('tolerance_c', 1.0)
        self.declare_parameter('min_area_px', 80)

        topic = self.get_parameter('image_topic').value
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.pub = self.create_publisher(Detection2DArray, 'thermal_detections', 10)
        self.get_logger().info(f'Listening on {topic}; publishing -> /thermal_detections')

    def _image_to_celsius(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
        if msg.encoding == 'mono16':
            vmin, vmax = float(img.min()), float(img.max())
            if 2000 <= vmin <= 6000 and 2000 <= vmax <= 6000:
                kelvin = img / 10.0
            elif 20000 <= vmin <= 60000 and 20000 <= vmax <= 60000:
                kelvin = img / 100.0
            else:
                kelvin = img
            return kelvin_to_celsius(kelvin)
        elif msg.encoding == 'mono8':
            return 15.0 + (img * (30.0 / 255.0))
        else:
            return kelvin_to_celsius(img / 10.0)

    def cb(self, msg: Image):
        try:
            celsius = self._image_to_celsius(msg)
        except Exception as e:
            self.get_logger().warn(f'Convert failed: {e}')
            return

        human_t = float(self.get_parameter('human_temp_c').value)
        deer_t  = float(self.get_parameter('deer_temp_c').value)
        tol     = float(self.get_parameter('tolerance_c').value)

        human_mask = (celsius >= human_t - tol) & (celsius <= human_t + tol)
        deer_mask  = (celsius >= deer_t  - tol) & (celsius <= deer_t  + tol)

        out = Detection2DArray()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id or 'thermal_frame'

        def add(mask, label, target):
            m = (mask.astype(np.uint8) * 255)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
            contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                x,y,w,h = cv2.boundingRect(c)
                if w*h < int(self.get_parameter('min_area_px').value): continue
                roi = celsius[y:y+h, x:x+w]
                mean_t = float(np.mean(roi)) if roi.size else target
                conf = max(0.0, 1.0 - abs(mean_t - target)/(2*tol if tol>0 else 1.0))

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label
                hyp.hypothesis.score = conf

                d = Detection2D()
                d.header = out.header
                d.results.append(hyp)
                d.bbox.center.position.x = float(x + w/2.0)
                d.bbox.center.position.y = float(y + h/2.0)
                d.bbox.size_x = float(w)
                d.bbox.size_y = float(h)
                out.detections.append(d)

        add(human_mask, 'human', human_t)
        add(deer_mask,  'deer',  deer_t)

        if out.detections:
            self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ThermalClassifier())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
