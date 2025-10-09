#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2

class ThermalTest(Node):
    def __init__(self):
        super().__init__('thermal_test')

        # ---- Topic & QoS ----
        self.declare_parameter('image_topic', '/model/husky/thermal/image')
        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.sub = self.create_subscription(Image, topic, self.cb, qos)
        self.pub = self.create_publisher(String, '/thermal_simple', 10)

        # ---- Kelvin-based params ----
        # If your sim outputs K*100 (Ignition default), keep kelvin_x100=True.
        self.declare_parameter('kelvin_x100', True)
        self.declare_parameter('sensor_min_k', 0.0)     # 0 = disabled
        self.declare_parameter('sensor_max_k', 0.0)     # 0 = disabled
        self.declare_parameter('human_temp_k', 310.15)  # ~37°C
        self.declare_parameter('animal_temp_k', 311.65) # ~38.5°C
        self.declare_parameter('tol_k', 0.2)            # +/- band in Kelvin
        self.declare_parameter('min_area_px', 80)
        self.declare_parameter('morph_kernel', 3)

        self.kelvin_x100 = bool(self.get_parameter('kelvin_x100').value)
        self.sensor_min_k = float(self.get_parameter('sensor_min_k').value)
        self.sensor_max_k = float(self.get_parameter('sensor_max_k').value)
        self.human_k = float(self.get_parameter('human_temp_k').value)
        self.animal_k = float(self.get_parameter('animal_temp_k').value)
        self.tol_k = float(self.get_parameter('tol_k').value)
        self.min_area = int(self.get_parameter('min_area_px').value)
        k = int(self.get_parameter('morph_kernel').value)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

        self.get_logger().info(
            f"🔍 Listening on {topic} | human={self.human_k}K animal={self.animal_k}K tol=±{self.tol_k}K"
        )

    def _to_kelvin(self, msg: Image, img_u16: np.ndarray) -> np.ndarray:
        if msg.is_bigendian:
            img_u16 = img_u16.byteswap()
        if self.kelvin_x100:
            return img_u16.astype(np.float32) / 100.0
        else:
            return img_u16.astype(np.float32)

    # --- replace your cb() with this version ---
    def cb(self, msg: Image):
        # 1) Rebuild uint16 image (respect endianness)
        img_u16 = np.frombuffer(msg.data, dtype=np.uint16)
        if img_u16.size != msg.height * msg.width:
            return
        img_u16 = img_u16.reshape((msg.height, msg.width))
        if msg.is_bigendian:
            img_u16 = img_u16.byteswap()

        # 2) Convert to Kelvin (Gazebo default is K*100) -> temp_k
        temp_k = (img_u16.astype(np.float32) / 100.0) if self.kelvin_x100 else img_u16.astype(np.float32)

        # (optional) clip for stability / visualization
        if self.sensor_min_k > 0.0 and self.sensor_max_k > 0.0 and self.sensor_max_k > self.sensor_min_k:
            temp_k = np.clip(temp_k, self.sensor_min_k, self.sensor_max_k)

        # 3) Build a broad foreground mask so we find blobs once
        # Use a permissive range around both targets
        k_min = min(self.human_k, self.animal_k) - max(self.tol_k, 1.0)
        k_max = max(self.human_k, self.animal_k) + max(self.tol_k, 1.0)
        fg = ((temp_k >= k_min) & (temp_k <= k_max)).astype(np.uint8) * 255
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, self.kernel)

        contours, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if w * h < self.min_area:
                continue

            roi_k = temp_k[y:y+h, x:x+w]
            mean_k = float(np.mean(roi_k))
            mean_c = mean_k - 273.15

            # 4) Nearest-prototype classification
            d_h = abs(mean_k - self.human_k)
            d_a = abs(mean_k - self.animal_k)
            # Accept only if it's close enough to at least one class
            if d_h <= self.tol_k or d_a <= self.tol_k:
                label = "human_like" if d_h <= d_a else "animal_like"
            else:
                # too far from both; skip
                continue

            cx, cy = int(x + w/2), int(y + h/2)
            out = f"{label} at ({cx},{cy}), box={w}x{h}, avg={mean_c:.1f}°C ({mean_k:.2f}K), ΔH={d_h:.2f}K ΔA={d_a:.2f}K"
            self.pub.publish(String(data=out))
            self.get_logger().info(out)


def main():
    rclpy.init()
    rclpy.spin(ThermalTest())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
