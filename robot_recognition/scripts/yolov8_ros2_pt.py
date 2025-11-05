#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import os

from yolov8_msgs.msg import InferenceResult, Yolov8Inference

bridge = CvBridge()

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        # -------------------------------
        # Load YOLOv8 model
        # -------------------------------
        package_share = get_package_share_directory('robot_recognition')
        model_path = os.path.join(package_share, 'models', 'final.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"✅ YOLOv8 model loaded from: {model_path}")

        self.frame_idx = 0

        # -------------------------------
        # ROS topics
        # -------------------------------
        self.subscription = self.create_subscription(
            Image,
            '/robot1/camera/image',  # ✅ your robot’s RGB camera
            self.camera_callback,
            10)
        self.subscription

        # Publishers
        self.yolov8_pub = self.create_publisher(Yolov8Inference, '/Yolov8_Inference', 1)
        self.img_pub = self.create_publisher(Image, '/inference_result', 1)

        # Data holder
        self.yolov8_inference = Yolov8Inference()

    def camera_callback(self, data):

        self.frame_idx += 1
        if self.frame_idx % 6 != 0:   # process ~every 6th frame (reduce load)
            return
        
        try:
            # -------------------------------
            # Convert ROS Image → OpenCV
            # -------------------------------
            img = bridge.imgmsg_to_cv2(data, "bgr8")

            # -------------------------------
            # Run YOLOv8 inference
            # -------------------------------
            results = self.model(img)

            # -------------------------------
            # Build custom message
            # -------------------------------
            self.yolov8_inference.header.frame_id = data.header.frame_id
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # Confidence from YOLO
                    try:
                        conf = float(box.conf[0].item())
                    except Exception:
                        # Fallback if conf is a scalar tensor
                        conf = float(box.conf)

                    b = box.xyxy[0].to('cpu').detach().numpy().copy()
                    c = int(box.cls)
                    class_name = self.model.names[c]

                    # Publish only rifle above 0.75 confidence to trigger hunter logic downstream
                    if class_name.lower() == 'rifle' and conf >= 0.75:
                        inference_result = InferenceResult()
                        inference_result.class_name = class_name
                        inference_result.top = int(b[0])
                        inference_result.left = int(b[1])
                        inference_result.bottom = int(b[2])
                        inference_result.right = int(b[3])
                        self.yolov8_inference.yolov8_inference.append(inference_result)

            # -------------------------------
            # Annotate image and publish
            # -------------------------------
            annotated_frame = results[0].plot()
            img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            img_msg.header = data.header  # ✅ keep same camera frame

            # Publish both messages
            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)

            # Clear for next frame
            self.yolov8_inference.yolov8_inference.clear()

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
