#!/usr/bin/env python3
import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
import tf2_ros
from rclpy.time import Time


class ThermalClassifier(Node):
    """Detect a human-like heat source and report bearing.

    Publishes:
    - /thermal_human: std_msgs/String summary (pixel, az/el, box, temps)
    - /thermal/human_dir: geometry_msgs/PointStamped unit vector in camera frame
    """

    def __init__(self) -> None:
        super().__init__('thermal_classifier')

        # ---- Topic & QoS ----
        self.declare_parameter('image_topic', '/robot1/thermal/image')
        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(Image, topic, self.cb, qos)
        self.pub_txt = self.create_publisher(String, '/thermal_human', 10)
        self.pub_dir = self.create_publisher(PointStamped, '/thermal/human_dir', 10)
        self.pub_world = self.create_publisher(PointStamped, '/thermal/human_world', 10)

        # ---- Parameters ----
        self.declare_parameter('scale_hint', 'auto')        # auto|x100|x10|x1
        self.declare_parameter('kelvin_x100', True)         # legacy: divide by 100 for Kelvin
        self.declare_parameter('human_temp_k', 309.15)      # ~36 °C (world_gen patch)
        self.declare_parameter('animal_temp_k', 311.65)     # ~38.5 °C
        self.declare_parameter('tol_k', 3.0)                # ± band in Kelvin
        self.declare_parameter('min_area_px', 20)
        self.declare_parameter('morph_kernel', 1)
        self.declare_parameter('horizontal_fov', 1.3963)    # ~80°; must match thermal camera
        self.declare_parameter('debug_every', 10)
        self.declare_parameter('enable_fallback', True)
        self.declare_parameter('min_fraction_in_band', 0.25)
        self.declare_parameter('min_delta_k', 4.0)          # min peak above ambient mean
        # World projection params
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('target_plane_z', 1.0)       # intersection plane height (m)
        self.declare_parameter('min_world_range_m', 1.5)    # reject hits too close to robot

        # ---- Load params ----
        self.scale_hint = str(self.get_parameter('scale_hint').value)
        self.kelvin_x100 = bool(self.get_parameter('kelvin_x100').value)
        self.human_k = float(self.get_parameter('human_temp_k').value)
        self.animal_k = float(self.get_parameter('animal_temp_k').value)
        self.tol_k = float(self.get_parameter('tol_k').value)
        self.min_area = int(self.get_parameter('min_area_px').value)
        k = max(1, int(self.get_parameter('morph_kernel').value))
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        self.h_fov = float(self.get_parameter('horizontal_fov').value)
        self.debug_every = max(1, int(self.get_parameter('debug_every').value))
        self.enable_fallback = bool(self.get_parameter('enable_fallback').value)
        self.min_fraction_in_band = float(self.get_parameter('min_fraction_in_band').value)
        self.min_delta_k = float(self.get_parameter('min_delta_k').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.target_plane_z = float(self.get_parameter('target_plane_z').value)
        self.min_world_range_m = float(self.get_parameter('min_world_range_m').value)

        # TF buffer/listener to transform camera rays to world
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # debug mask publisher (no cv_bridge dependency)
        self.pub_mask = self.create_publisher(Image, '/thermal/debug_mask', 1)

        self.frame_count = 0
        self.get_logger().info(
            f"thermal_classifier listening on {topic} | human={self.human_k:.2f}K ±{self.tol_k:.2f}K | scale_hint={self.scale_hint}"
        )

    # ---- helpers ----
    def _auto_scale(self, raw_max: float) -> float:
        if self.scale_hint == 'x100':
            return 100.0
        if self.scale_hint == 'x10':
            return 10.0
        if self.scale_hint == 'x1':
            return 1.0
        # auto
        if raw_max > 20000:
            return 100.0
        if raw_max > 2000:
            return 10.0
        return 1.0

    def _to_kelvin(self, msg: Image) -> Optional[np.ndarray]:
        # Expecting 16-bit mono (L16) from Ignition; handle endianness.
        buf = np.frombuffer(msg.data, dtype=np.uint16)
        if buf.size != msg.height * msg.width:
            self.get_logger().warn(f"thermal img size mismatch: {buf.size} vs {msg.height}*{msg.width}")
            return None
        img_u16 = buf.reshape((msg.height, msg.width))
        if msg.is_bigendian:
            img_u16 = img_u16.byteswap()

        raw_max = float(img_u16.max())
        # Prefer explicit kelvin_x100 when provided; otherwise auto
        if self.scale_hint == 'auto' and self.kelvin_x100:
            scale = 100.0
        else:
            scale = self._auto_scale(raw_max)

        temp_k = img_u16.astype(np.float32) / scale

        if (self.frame_count % self.debug_every) == 0:
            self.get_logger().info(
                f"[thermal] enc=L16 raw_max={raw_max:.0f} scale=1/{scale:.0f} -> K[min/mean/max]={temp_k.min():.2f}/{temp_k.mean():.2f}/{temp_k.max():.2f}"
            )
        return temp_k

    # ---- callback ----
    def cb(self, msg: Image) -> None:
        temp_k = self._to_kelvin(msg)
        if temp_k is None:
            return
        self.frame_count += 1

        # Broad mask around human temp range to find blobs
        k_min = self.human_k - self.tol_k
        k_max = self.human_k + self.tol_k
        mask = ((temp_k >= k_min) & (temp_k <= k_max)).astype(np.uint8) * 255
        if self.kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Publish debug mask
        try:
            img_msg = Image()
            img_msg.header = msg.header
            img_msg.height, img_msg.width = mask.shape
            img_msg.encoding = 'mono8'
            img_msg.is_bigendian = 0
            img_msg.step = int(mask.shape[1])
            img_msg.data = mask.tobytes()
            self.pub_mask.publish(img_msg)
        except Exception:
            pass

        if not contours and not self.enable_fallback:
            return

        H, W = temp_k.shape
        cx0, cy0 = (W - 1) / 2.0, (H - 1) / 2.0
        h_fov = self.h_fov
        v_fov = h_fov * (H / float(W))

        ambient_mean = float(np.mean(temp_k))

        # Pick the largest blob as the likely human
        chosen = None
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if w * h < self.min_area:
                continue

            roi = temp_k[y:y+h, x:x+w]
            roi_mask = mask[y:y+h, x:x+w] > 0
            if not np.any(roi_mask):
                continue
            # stats inside the in-band area only
            band_vals = roi[roi_mask]
            mean_k = float(np.mean(band_vals))
            min_k = float(np.min(band_vals))
            max_k = float(np.max(band_vals))
            frac_in_band = float(band_vals.size) / float(w * h)
            # require sufficient coverage and a peak above ambient by min_delta_k
            if frac_in_band < self.min_fraction_in_band:
                continue
            if (max_k - ambient_mean) < self.min_delta_k:
                continue

            # Center pixel and bearing
            cx, cy = int(x + w/2), int(y + h/2)
            az = (cx - cx0) * (h_fov / float(W))
            el = -(cy - cy0) * (v_fov / float(H))

            # Unit direction vector in camera frame (z forward)
            cz = math.cos(el) * math.cos(az)
            cxv = math.cos(el) * math.sin(az)
            cyv = math.sin(el)

            # Compute world position by intersecting camera ray with z = target_plane_z
            world_pt = self._ray_to_world(msg.header, cxv, cyv, cz)
            if world_pt is not None:
                wx, wy, wz = world_pt
                psw = PointStamped()
                psw.header = msg.header
                psw.header.frame_id = self.world_frame
                psw.point = Point(x=float(wx), y=float(wy), z=float(wz))
                self.pub_world.publish(psw)
                # Simple message following actual world location (rounded to ints)
                txt = f"human detected at {int(round(wx))},{int(round(wy))},{int(round(wz))}"
            else:
                txt = (
                    f"No Human: pixel=({cx},{cy}) az={math.degrees(az):.1f}deg el={math.degrees(el):.1f}deg, "
                    f"box={w}x{h}, band_avg={mean_k:.2f}K [{min_k:.2f},{max_k:.2f}], frac={frac_in_band:.2f}, Δamb={max_k-ambient_mean:.2f}K"
                )
            self.pub_txt.publish(String(data=txt))
            self.get_logger().info(txt)

            ps = PointStamped()
            ps.header = msg.header
            ps.point.x = cxv
            ps.point.y = cyv
            ps.point.z = cz
            self.pub_dir.publish(ps)
            chosen = True
            break

        # Fallback: if no blobs survived morphology/area, use hottest pixel in frame
        if not chosen and self.enable_fallback:
            yx = np.unravel_index(np.argmax(temp_k), temp_k.shape)
            yy, xx = int(yx[0]), int(yx[1])
            max_k = float(temp_k[yy, xx])
            # only fire fallback if hottest pixel is within the target band and above ambient by min_delta_k
            if (k_min <= max_k <= k_max) and ((max_k - ambient_mean) >= self.min_delta_k):
                az = (xx - cx0) * (h_fov / float(W))
                el = -(yy - cy0) * (v_fov / float(H))
                cz = math.cos(el) * math.cos(az)
                cxv = math.cos(el) * math.sin(az)
                cyv = math.sin(el)

                world_pt = self._ray_to_world(msg.header, cxv, cyv, cz)
                if world_pt is not None:
                    wx, wy, wz = world_pt
                    psw = PointStamped()
                    psw.header = msg.header
                    psw.header.frame_id = self.world_frame
                    psw.point = Point(x=float(wx), y=float(wy), z=float(wz))
                    self.pub_world.publish(psw)
                    txt = f"human detected at {int(round(wx))},{int(round(wy))},{int(round(wz))}"
                else:
                    txt = (
                        f"human detected (fallback): pixel=({xx},{yy}) az={math.degrees(az):.1f}deg el={math.degrees(el):.1f}deg, "
                        f"hot={max_k:.2f}K, Δamb={max_k-ambient_mean:.2f}K"
                    )
                self.pub_txt.publish(String(data=txt))
                self.get_logger().info(txt)
                ps = PointStamped()
                ps.header = msg.header
                ps.point.x = cxv
                ps.point.y = cyv
                ps.point.z = cz
                self.pub_dir.publish(ps)

    def _ray_to_world(self, header, cxv: float, cyv: float, czv: float):
        """Transform camera-frame unit ray to world frame and intersect with z = target_plane_z.

        Returns (wx, wy, wz) in a resolved world-like frame, or None if TF is unavailable/degenerate.
        """
        src_frame = header.frame_id if header.frame_id else 'thermal_link'
        # Use latest available TF (time=0) to avoid stamp mismatch issues from bridged images
        stamp = Time()

        # Resolve a usable world-like frame
        world_candidates = [self.world_frame, 'world', 'map', 'odom']
        target_frame = None
        for wf in world_candidates:
            if not wf:
                continue
            try:
                _ = self.tf_buffer.lookup_transform(wf, src_frame, stamp)
                target_frame = wf
                break
            except Exception:
                continue
        if target_frame is None:
            return None

        try:
            tf = self.tf_buffer.lookup_transform(target_frame, src_frame, stamp)
        except Exception:
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        # Quaternion to rotation matrix (camera -> world)
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),       2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),       1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
        ], dtype=np.float64)

        cam_pos = np.array([tx, ty, tz], dtype=np.float64)
        cam_dir_local = np.array([cxv, cyv, czv], dtype=np.float64)
        ray_world = R @ cam_dir_local
        dz = ray_world[2]
        if abs(dz) < 1e-6:
            return None
        t = (self.target_plane_z - cam_pos[2]) / dz
        if t <= 0:
            return None
        hit = cam_pos + t * ray_world
        # Enforce minimum horizontal range from camera to avoid self-detections (robot body)
        horiz = hit[:2] - cam_pos[:2]
        if float(np.linalg.norm(horiz)) < self.min_world_range_m:
            return None
        # Update to the actually used frame
        self.world_frame = target_frame
        return float(hit[0]), float(hit[1]), float(hit[2])


def main() -> None:
    rclpy.init()
    rclpy.spin(ThermalClassifier())
    rclpy.shutdown()


if __name__ == '__main__':
    main()