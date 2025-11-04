#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class ThermalDetection(Node):
    def __init__(self):
        super().__init__('thermal_detection')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/robot1/thermal/image',
            self.image_callback,
            10
        )
        self.get_logger().info("🔥 Thermal detection node started — listening to /robot1/thermal/image")

    def image_callback(self, msg):
        # Convert the ROS2 Image (mono16) to numpy array
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16').astype(np.float32)

        # Gazebo thermal camera values are in Kelvin
        temp_kelvin = frame/100

        # Compute frame statistics
        min_temp = np.min(temp_kelvin)
        mean_temp = np.mean(temp_kelvin)
        max_temp = np.max(temp_kelvin)

        self.get_logger().info(
            f"🌡 Frame stats → min: {min_temp:.2f} K | mean: {mean_temp:.2f} K | max: {max_temp:.2f} K"
        )

        # Threshold: detect anything warmer than ~308 K (~35 °C)
        human_mask = (temp_kelvin >= 308.0) & (temp_kelvin < 310.15) # up to ~37 °C for human body temp

        if np.any(human_mask):
            # Get centroid of the detected hot region
            coords = np.column_stack(np.where(human_mask))
            y, x = np.mean(coords, axis=0)

            # Get temperature of hottest spot
            max_temp = np.max(temp_kelvin)

            # Normalize to image coordinates
            height, width = frame.shape
            x_norm = (x - width / 2) / width
            y_norm = (y - height / 2) / height

            self.get_logger().info(
                f"👤 Human detected at (x={x_norm:.2f}, y={y_norm:.2f}) | Peak ≈ {max_temp:.2f} K"
            )

            # Optional visualization
            vis = cv2.applyColorMap(
                np.uint8(np.clip((temp_kelvin - 309.15) * 2, 0, 255)), cv2.COLORMAP_JET
            )
            # Create a red overlay where the human_mask is true
            overlay = vis.copy()
            overlay[human_mask] = [0, 0, 255]  # Pure red in BGR
            # Blend the overlay with the original visualization
            alpha = 0.6  # transparency factor (0–1)
            vis = cv2.addWeighted(overlay, alpha, vis, 1 - alpha, 0)
            cv2.circle(vis, (int(x), int(y)), 8, (255, 255, 255), 2)
            cv2.imshow("Thermal Detection", vis)
            cv2.waitKey(1)
        else:
            self.get_logger().info("No warm object detected.")

def main(args=None):
    rclpy.init(args=args)
    node = ThermalDetection()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
