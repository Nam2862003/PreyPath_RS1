#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from message_filters import ApproximateTimeSynchronizer, Subscriber
import image_geometry
from collections import deque

class ThermalDepthAuto(Node):
    def __init__(self):
        super().__init__('thermal_depth_auto')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        self.thermal_sub = Subscriber(self, Image, '/robot1/thermal/image')
        self.depth_sub = Subscriber(self, Image, '/robot1/camera/depth/image')
        self.thermal_info = Subscriber(self, CameraInfo, '/robot1/thermal/camera_info')
        self.depth_info = Subscriber(self, CameraInfo, '/robot1/camera/camera_info')

        self.human_uv_sub = self.create_subscription(Int32MultiArray, '/human_pixel_uv', self.human_uv_callback, 10)
        self.hunter_uv_sub = self.create_subscription(Int32MultiArray, '/hunter_pixel_uv', self.hunter_uv_callback, 10)
        self.gun_uv_sub = self.create_subscription(Int32MultiArray, '/gun_pixel_uv', self.gun_uv_callback, 10)

        # --- Publishers ---
        self.human_pose_pub = self.create_publisher(PointStamped, '/human_pose', 10)
        self.hunter_pose_pub = self.create_publisher(PointStamped, '/hunter_pose', 10)
        self.hunter_alert_pub = self.create_publisher(String, '/hunter_alert', 10)

        # --- Synchronizer ---
        self.ts = ApproximateTimeSynchronizer(
            [self.thermal_sub, self.depth_sub, self.thermal_info, self.depth_info],
            queue_size=10, slop=0.3
        )
        self.ts.registerCallback(self.sync_callback)

        self.camera_model = None

        # --- Detection lists ---
        self.last_human_uvs = []
        self.last_hunter_uvs = []
        self.last_gun_uvs = []

        # --- Rolling pose buffers ---
        self.human_poses = deque(maxlen=100)
        self.hunter_poses = deque(maxlen=100)

        self.human_avg_published = False
        self.hunter_avg_published = False

        self.get_logger().info("🔥 ThermalDepthAuto with averaging ready!")

    # ----------------------------------------------------------
    # YOLO Callbacks
    # ----------------------------------------------------------
    def human_uv_callback(self, msg):
        self.last_human_uvs = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]

    def hunter_uv_callback(self, msg):
        self.last_hunter_uvs = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]

    def gun_uv_callback(self, msg):
        self.last_gun_uvs = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]

    # ----------------------------------------------------------
    # Core synchronized callback
    # ----------------------------------------------------------
    def sync_callback(self, thermal_msg, depth_msg, tinfo, dinfo):
        try:
            thermal = self.bridge.imgmsg_to_cv2(thermal_msg, 'mono16').astype(np.float32) / 100.0
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)

            if self.camera_model is None:
                self.camera_model = image_geometry.PinholeCameraModel()
                self.camera_model.fromCameraInfo(dinfo)
                self.get_logger().info("📷 Camera model initialized.")

            HUMAN_RANGE = (308.0, 310.5)
            HUNTER_RANGE = (308.0, 310.5)
            GUN_RANGE = (314.0, 318.0)

            # Process Humans
            for (u, v) in self.last_human_uvs:
                if not self._valid_pixel(u, v, depth): 
                    continue
                temp = float(thermal[int(v), int(u)])
                if HUMAN_RANGE[0] <= temp <= HUMAN_RANGE[1]:
                    pt = self.project_to_world(u, v, depth, dinfo)
                    if pt:
                        self.human_poses.append([pt.point.x, pt.point.y, pt.point.z])
                        if len(self.human_poses) >= 50 and not self.human_avg_published:
                            avg_pose = np.mean(self.human_poses, axis=0)
                            self.publish_average(avg_pose, self.human_pose_pub, "👤 Human")
                            self.human_avg_published = True

            # Process Hunters
            for (u, v) in self.last_hunter_uvs:
                if not self._valid_pixel(u, v, depth): 
                    continue
                temp = float(thermal[int(v), int(u)])
                if HUNTER_RANGE[0] <= temp <= HUNTER_RANGE[1]:
                    pt = self.project_to_world(u, v, depth, dinfo)
                    if pt:
                        self.hunter_poses.append([pt.point.x, pt.point.y, pt.point.z])
                        if len(self.hunter_poses) >= 50 and not self.hunter_avg_published:
                            avg_pose = np.mean(self.hunter_poses, axis=0)
                            self.publish_average(avg_pose, self.hunter_pose_pub, "⚠️ Hunter")
                            self.hunter_avg_published = True
                            alert = String()
                            alert.data = "⚠️ Hunter detected (averaged position)"
                            self.hunter_alert_pub.publish(alert)

        except Exception as e:
            self.get_logger().error(f"❌ sync_callback error: {e}")

    # ----------------------------------------------------------
    # Helper functions
    # ----------------------------------------------------------
    def _valid_pixel(self, u, v, depth_img):
        h, w = depth_img.shape
        if u < 0 or v < 0 or u >= w or v >= h:
            return False
        d = float(depth_img[int(v), int(u)])
        return not np.isnan(d) and 0.1 < d < 10.0

    def project_to_world(self, u, v, depth_img, info):
        try:
            d = float(depth_img[int(v), int(u)])
            ray = self.camera_model.projectPixelTo3dRay((u, v))
            X_cam, Y_cam, Z_cam = ray[0]*d, ray[1]*d, d
            pt_cam = PointStamped()
            pt_cam.header.frame_id = info.header.frame_id
            pt_cam.header.stamp = self.get_clock().now().to_msg()
            pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = X_cam, Y_cam, Z_cam
            tf = self.tf_buffer.lookup_transform('map', pt_cam.header.frame_id, rclpy.time.Time())
            return do_transform_point(pt_cam, tf)
        except Exception as e:
            self.get_logger().warn(f"TF/project error: {e}")
            return None

    def publish_average(self, avg_xyz, pub, label):
        avg_point = PointStamped()
        avg_point.header.frame_id = 'map'
        avg_point.header.stamp = self.get_clock().now().to_msg()
        avg_point.point.x, avg_point.point.y, avg_point.point.z = avg_xyz
        pub.publish(avg_point)
        self.get_logger().info(f"✅ Averaged {label} pose published → map=({avg_xyz[0]:.2f}, {avg_xyz[1]:.2f}, {avg_xyz[2]:.2f})")

def main():
    rclpy.init()
    node = ThermalDepthAuto()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
