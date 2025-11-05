#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from message_filters import ApproximateTimeSynchronizer, Subscriber
import image_geometry


class ThermalDepthAuto(Node):
    def __init__(self):
        super().__init__('thermal_depth_auto_static')
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

        # --- Publishers ---
        self.human_pose_pub = self.create_publisher(PoseArray, '/human_pose', 10)
        self.hunter_pose_pub = self.create_publisher(PoseArray, '/hunter_pose', 10)
        self.hunter_alert_pub = self.create_publisher(String, '/hunter_alert', 10)

        # --- Synchronizer ---
        self.ts = ApproximateTimeSynchronizer(
            [self.thermal_sub, self.depth_sub, self.thermal_info, self.depth_info],
            queue_size=10, slop=0.3
        )
        self.ts.registerCallback(self.sync_callback)

        # --- Camera model ---
        self.camera_model = None

        # --- Cache ---
        self.detected_humans = {}   # {id: np.array([x, y, z])}
        self.detected_hunters = {}  # {id: np.array([x, y, z])}

        self.get_logger().info("🔥 ThermalDepthAuto Static Mode ready!")

    # ----------------------------------------------------------
    # YOLO Callbacks
    # ----------------------------------------------------------
    def human_uv_callback(self, msg):
        self.last_human_uvs = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]

    def hunter_uv_callback(self, msg):
        self.last_hunter_uvs = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]

    # ----------------------------------------------------------
    # Main synchronized callback
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

            # --- Detect Humans ---
            for i, (u, v) in enumerate(getattr(self, 'last_human_uvs', [])):
                if i in self.detected_humans:  # already frozen
                    continue
                if not self._valid_pixel(u, v, depth):
                    continue
                temp = float(thermal[int(v), int(u)])
                if HUMAN_RANGE[0] <= temp <= HUMAN_RANGE[1]:
                    pt = self.project_to_world(u, v, depth, dinfo)
                    if pt:
                        self.detected_humans[i] = np.array([pt.point.x, pt.point.y, pt.point.z])
                        self.get_logger().info(
                            f"🧍 Human {i} locked at map=({pt.point.x:.2f}, {pt.point.y:.2f}, {pt.point.z:.2f})"
                        )

            # --- Detect Hunters ---
            for i, (u, v) in enumerate(getattr(self, 'last_hunter_uvs', [])):
                if i in self.detected_hunters:  # already frozen
                    continue
                if not self._valid_pixel(u, v, depth):
                    continue
                temp = float(thermal[int(v), int(u)])
                if HUNTER_RANGE[0] <= temp <= HUNTER_RANGE[1]:
                    pt = self.project_to_world(u, v, depth, dinfo)
                    if pt:
                        self.detected_hunters[i] = np.array([pt.point.x, pt.point.y, pt.point.z])
                        self.get_logger().info(
                            f"🎯 Hunter {i} locked at map=({pt.point.x:.2f}, {pt.point.y:.2f}, {pt.point.z:.2f})"
                        )

            # --- Publish ---
            self.publish_pose_array(self.detected_humans, self.human_pose_pub, "👤 Human")
            self.publish_pose_array(self.detected_hunters, self.hunter_pose_pub, "⚠️ Hunter")

            # --- Alert ---
            if self.detected_hunters:
                alert = String()
                alert.data = f"⚠️ {len(self.detected_hunters)} Hunter(s) detected!"
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

            # Project once at detection time only
            tf = self.tf_buffer.lookup_transform(
                'map', info.header.frame_id, rclpy.time.Time(seconds=0)
            )
            return do_transform_point(pt_cam, tf)

        except Exception as e:
            self.get_logger().warn(f"TF/project error: {e}")
            return None

    def publish_pose_array(self, detections, pub, label):
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for pos in detections.values():
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos
            pose_array.poses.append(pose)

        if pose_array.poses:
            pub.publish(pose_array)
            self.get_logger().info(f"✅ Published {len(pose_array.poses)} locked {label} poses.")


def main():
    rclpy.init()
    node = ThermalDepthAuto()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
