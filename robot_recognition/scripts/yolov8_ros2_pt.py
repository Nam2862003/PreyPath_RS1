#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os
from yolov8_msgs.msg import InferenceResult, Yolov8Inference


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # -------------------------------
        # Load YOLOv8 model
        # -------------------------------
        pkg_share = get_package_share_directory('robot_recognition')
        model_path = os.path.join(pkg_share, 'models', 'final.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"✅ YOLOv8 model loaded from: {model_path}")

        self.bridge = CvBridge()
        self.frame_idx = 0
        self.hunter_threshold_px = 80   # max pixel distance between gun & human
        self.duplicate_threshold_px = 40  # distance under which two humans = same

        # -------------------------------
        # ROS topics
        # -------------------------------
        self.subscription = self.create_subscription(
            Image, '/robot1/camera/image', self.camera_callback, 10
        )
        self.img_pub = self.create_publisher(Image, '/inference_result', 1)
        self.yolov8_pub = self.create_publisher(Yolov8Inference, '/Yolov8_Inference', 1)
        self.human_uv_pub = self.create_publisher(Int32MultiArray, '/human_pixel_uv', 10)
        self.gun_uv_pub = self.create_publisher(Int32MultiArray, '/gun_pixel_uv', 10)
        self.hunter_uv_pub = self.create_publisher(Int32MultiArray, '/hunter_pixel_uv', 10)
        self.hunter_alert_pub = self.create_publisher(String, '/hunter_alert', 10)

        self.yolov8_inference = Yolov8Inference()

    def camera_callback(self, data):
        self.frame_idx += 1
        if self.frame_idx % 2 != 0:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            results = self.model(img)
            self.yolov8_inference.header.frame_id = data.header.frame_id
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

            uv_humans, uv_guns = [], []

            # -------------------------------
            # Parse detections
            # -------------------------------
            for r in results:
                for box in r.boxes:
                    b = box.xyxy[0].cpu().numpy()
                    c = int(box.cls)
                    cls_name = self.model.names[c]
                    conf = float(box.conf.cpu().numpy()) * 100
                    u = int((b[0] + b[2]) / 2)
                    v = int((b[1] + b[3]) / 2)

                    # record in inference msg
                    inf = InferenceResult()
                    inf.class_name = cls_name
                    inf.top, inf.left, inf.bottom, inf.right = map(int, [b[1], b[0], b[3], b[2]])
                    self.yolov8_inference.yolov8_inference.append(inf)

                    if cls_name.lower() == "person" and conf > 65:
                        uv_humans.append(((u, v), conf))
                        # self.get_logger().info(f"👤 Human ({conf:.1f}%) at ({u},{v})")
                    elif cls_name.lower() in ["rifle", "gun", "weapon"] and conf > 65:
                        uv_guns.append(((u, v), conf))
                        # self.get_logger().info(f"🔫 Gun ({conf:.1f}%) at ({u},{v})")

            # -------------------------------
            # Detect & separate hunters
            # -------------------------------
            hunters, safe_humans = [], []

            for ((uh, vh), conf_h) in uv_humans:
                is_hunter = False
                for ((ug, vg), conf_g) in uv_guns:
                    dist = np.sqrt((uh - ug) ** 2 + (vh - vg) ** 2)
                    if dist <= self.hunter_threshold_px and conf_h > 70 and conf_g > 70:
                        is_hunter = True
                        hunters.append(((uh, vh), conf_h))
                        alert = String()
                        alert.data = f"⚠️ Hunter detected! ({uh, vh}) dist={dist:.1f}px | Human={conf_h:.1f}% | Gun={conf_g:.1f}%"
                        self.hunter_alert_pub.publish(alert)
                        self.get_logger().warn(alert.data)
                        break  # found gun close enough

                # only add to safe_humans if not near any gun
                if not is_hunter:
                    safe_humans.append(((uh, vh), conf_h))

            # -------------------------------
            # Remove near-duplicate humans (overlapping boxes)
            # -------------------------------
            filtered_humans = []
            for ((uh, vh), conf_h) in safe_humans:
                if not any(np.sqrt((uh - x) ** 2 + (vh - y) ** 2) < self.duplicate_threshold_px
                           for ((x, y), _) in filtered_humans):
                    filtered_humans.append(((uh, vh), conf_h))

    
            # -------------------------------
            # Publish results
            # -------------------------------
            annotated = results[0].plot()
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header = data.header
            self.img_pub.publish(img_msg)

            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

            # publish humans (non-hunters)
            if filtered_humans:
                msg = Int32MultiArray()
                msg.data = [val for (u, v), _ in filtered_humans for val in (u, v)]
                self.human_uv_pub.publish(msg)
                # 🧍 Log all normal humans
                self.get_logger().info("🧍 Normal Humans detected:")
                for (u, v), conf in filtered_humans:
                    self.get_logger().info(f"Person at ({u},{v})  conf={conf:.1f}%")

            # publish guns
            if uv_guns:
                msg = Int32MultiArray()
                msg.data = [val for (u, v), _ in uv_guns for val in (u, v)]
                self.gun_uv_pub.publish(msg)

            # publish hunters
            if hunters:
                msg = Int32MultiArray()
                msg.data = [val for (u, v), _ in hunters for val in (u, v)]
                self.hunter_uv_pub.publish(msg)
                # 🎯 Log all hunter positions
                self.get_logger().warn("🎯 HUNTERS DETECTED:")
                for (u, v), conf in hunters:
                    self.get_logger().warn(f"⚠️ Hunter at ({u},{v})  conf={conf:.1f}%")

            # 🧩 If nothing detected at all
            if not filtered_humans and not hunters:
                self.get_logger().info("🚫 No people or hunters detected this frame.")


        except Exception as e:
            self.get_logger().error(f"❌ YOLO callback error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    node.get_logger().info("🚀 YOLOv8 ROS2 node started. Listening to /robot1/camera/image ...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
