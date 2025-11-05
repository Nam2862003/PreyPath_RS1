#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from message_filters import ApproximateTimeSynchronizer, Subscriber
import image_geometry


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
        self.yolo_uv_sub = self.create_subscription(Int32MultiArray, '/human_pixel_uv', self.uv_callback, 10)

        # --- Publishers ---
        self.pose_pub = self.create_publisher(PointStamped, '/human_pose', 10)
        self.hunter_pose_pub = self.create_publisher(PointStamped, '/hunter_pose', 10)
        self.hunter_alert_pub = self.create_publisher(String, '/hunter_alert', 10)

        # --- Sync thermal + depth + info ---
        self.ts = ApproximateTimeSynchronizer(
            [self.thermal_sub, self.depth_sub, self.thermal_info, self.depth_info],
            queue_size=10,
            slop=0.2
        )
        self.ts.registerCallback(self.callback)

        self.camera_model = None
        self.last_uv = None  # from YOLO
        self.get_logger().info("🔥 Thermal-Depth Auto node ready.")

    def uv_callback(self, msg: Int32MultiArray):
        """Receive YOLO human pixel coordinates (u,v)."""
        if not msg.data or len(msg.data) < 2:
            self.last_uv = None
            return
        # Only use first human (you can extend to multiple)
        self.last_uv = (msg.data[0], msg.data[1])

    def callback(self, thermal_msg, depth_msg, tinfo, dinfo):
        # --- Convert to numpy ---
        thermal = self.bridge.imgmsg_to_cv2(thermal_msg, 'mono16').astype(np.float32) / 100.0
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)

        # --- Thermal temperature masks ---
        human_mask = (thermal >= 308.0) & (thermal < 310.5)
        hunter_mask = (thermal >= 320.0) & (thermal <= 324.0)

        # --- Camera model init ---
        if self.camera_model is None:
            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.fromCameraInfo(dinfo)
            self.get_logger().info("📷 Camera model initialized.")

        def project_to_world(u, v, depth_img, info, label, pub):
            d = float(depth_img[v, u])
            if np.isnan(d) or d <= 0.1:
                return None
            ray = self.camera_model.projectPixelTo3dRay((u, v))
            X_cam = ray[0] * d
            Y_cam = ray[1] * d
            Z_cam = d  # depth directly along optical axis

            pt_cam = PointStamped()
            pt_cam.header.frame_id = info.header.frame_id
            pt_cam.header.stamp = self.get_clock().now().to_msg()
            pt_cam.point.x = X_cam
            pt_cam.point.y = Y_cam
            pt_cam.point.z = Z_cam
            # self.pose_pub.publish(pt_cam)
            # self.get_logger().info(
            #     f"{label} → (u,v)=({u},{v}) depth={d:.2f} m | map=({pt_cam.point.x:.2f},{pt_cam.point.y:.2f},{pt_cam.point.z:.2f})"
            # )
            # return (u, v)
            try:
                tf = self.tf_buffer.lookup_transform('map', pt_cam.header.frame_id, rclpy.time.Time())
                pt_world = do_transform_point(pt_cam, tf)
                out = PointStamped()
                out.header.frame_id = 'map'
                out.header.stamp = self.get_clock().now().to_msg()
                out.point.x = pt_world.point.x
                out.point.y = pt_world.point.y
                out.point.z = pt_world.point.z
                self.pose_pub.publish(out)
                self.get_logger().info(
                    f"{label} → (u,v)=({u},{v}) depth={d:.2f} m | map=({pt_world.point.x:.2f},{pt_world.point.y:.2f},{pt_world.point.z:.2f})"
                )
                return (u, v)
            except Exception as e:
                self.get_logger().warn(f"TF failed: {e}")
                return None

        # --- Detection Logic ---
        has_thermal_human = np.any(human_mask)
        has_uv = self.last_uv is not None

        if has_thermal_human and not has_uv:
            self.get_logger().warn("🟡 Maybe Human detected (thermal only). Waiting for YOLO confirmation.")
        elif has_thermal_human and has_uv:
            u, v = self.last_uv
            self.get_logger().info(f"✅ Real Human confirmed! Using (u,v)=({u},{v}) for 3D projection.")
            project_to_world(u, v, depth, dinfo, "👤 Human", self.pose_pub)
        else:
            self.last_uv = None  # reset if no human

        # --- Hunter detection ---
        if np.any(hunter_mask):
            yy, xx = np.mean(np.column_stack(np.where(hunter_mask)), axis=0)
            u, v = int(xx), int(yy)
            project_to_world(u, v, depth, dinfo, "⚠️ Hunter", self.hunter_pose_pub)
            alert = String()
            alert.data = "⚠️ Hunter detected!"
            self.hunter_alert_pub.publish(alert)

        # --- Visualization ---
        # vis = cv2.applyColorMap(np.uint8(np.clip((thermal - 309.15) * 2, 0, 255)), cv2.COLORMAP_JET)
        # overlay = vis.copy()
        # overlay[human_mask] = [0, 0, 255]
        # overlay[hunter_mask] = [0, 255, 0]
        # vis = cv2.addWeighted(overlay, 0.6, vis, 0.4, 0)
        # if self.last_uv:
        #     cv2.circle(vis, self.last_uv, 8, (255, 255, 255), 2)
        # cv2.imshow("Thermal Detection", vis)
        # cv2.waitKey(1)


def main():
    rclpy.init()
    node = ThermalDepthAuto()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
