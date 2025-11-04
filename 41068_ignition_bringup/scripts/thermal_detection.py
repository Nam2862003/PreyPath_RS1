#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
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

        # Subscribers
        self.thermal_sub = Subscriber(self, Image, '/robot1/thermal/image')
        self.depth_sub = Subscriber(self, Image, '/robot1/camera/depth/image')
        self.thermal_info = Subscriber(self, CameraInfo, '/robot1/thermal/camera_info')
        self.depth_info = Subscriber(self, CameraInfo, '/robot1/camera/camera_info')
        self.pose_pub = self.create_publisher(PointStamped, '/human_pose', 10)
        # New: separate outputs for hunter detection
        self.hunter_pose_pub = self.create_publisher(PointStamped, '/hunter_pose', 10)
        self.hunter_alert_pub = self.create_publisher(String, '/hunter_alert', 10)

        # Synchronizer
        self.ts = ApproximateTimeSynchronizer(
            [self.thermal_sub, self.depth_sub, self.thermal_info, self.depth_info],
            queue_size=10,
            slop=0.2
        )
        self.ts.registerCallback(self.callback)

        self.camera_model = None
        self.get_logger().info("Thermal-Depth auto node ready.")

    def callback(self, thermal_msg, depth_msg, tinfo, dinfo):
        # Convert to numpy
        thermal = self.bridge.imgmsg_to_cv2(thermal_msg, 'mono16').astype(np.float32) / 100.0
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)

        # Detect hot regions
        human_mask = (thermal >= 308.0) & (thermal < 310.5)
        hunter_mask = (thermal >= 320.0) & (thermal <= 324.0)

        # Initialize camera model (only once)
        if self.camera_model is None:
            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.fromCameraInfo(dinfo)
            self.get_logger().info("Camera model initialized from depth_info.")

        def project_and_publish(mask, pose_pub, label):
            if not np.any(mask):
                return None
            yy, xx = np.mean(np.column_stack(np.where(mask)), axis=0)
            u, v = int(np.clip(xx, 0, depth.shape[1] - 1)), int(np.clip(yy, 0, depth.shape[0] - 1))
            d = float(depth[v, u])
            if np.isnan(d) or d <= 0.01:
                return None
            ray = self.camera_model.projectPixelTo3dRay((u, v))
            X_cam = ray[0] * d
            Y_cam = ray[1] * d
            Z_cam = ray[2] * d
            pt_cam = PointStamped()
            pt_cam.header.frame_id = dinfo.header.frame_id
            pt_cam.header.stamp = self.get_clock().now().to_msg()
            pt_cam.point.x = X_cam
            pt_cam.point.y = Y_cam
            pt_cam.point.z = Z_cam
            try:
                tf = self.tf_buffer.lookup_transform('map', pt_cam.header.frame_id, rclpy.time.Time())
                pt_world = do_transform_point(pt_cam, tf)
                out = PointStamped()
                out.header.frame_id = 'map'
                out.header.stamp = self.get_clock().now().to_msg()
                out.point.x = pt_world.point.z
                out.point.y = pt_world.point.y
                out.point.z = pt_world.point.x
                pose_pub.publish(out)
                self.get_logger().info(
                    f"{label} | pixel=({int(xx)}, {int(yy)}) | depth={d:.2f} m "
                    f"| map=({pt_world.point.z:.2f}, {pt_world.point.y:.2f}, {pt_world.point.x:.2f})"
                )
                return int(xx), int(yy)
            except Exception as e:
                self.get_logger().warn(f"TF transform failed: {e}")
                return None

        # Process each target independently
        human_px = project_and_publish(human_mask, self.pose_pub, "👤 Human")
        hunter_px = project_and_publish(hunter_mask, self.hunter_pose_pub, "⚠️ Hunter")

        # Sound/alert message when hunter detected
        if hunter_px is not None:
            alert = String()
            alert.data = "hunter detected"
            self.hunter_alert_pub.publish(alert)

        # --- Visualization (for debugging only) ---
        vis = cv2.applyColorMap(
            np.uint8(np.clip((thermal - 309.15) * 2, 0, 255)), cv2.COLORMAP_JET
        )
        overlay = vis.copy()
        # Red for human, Green for hunter
        overlay[human_mask] = [0, 0, 255]
        overlay[hunter_mask] = [0, 255, 0]
        vis = cv2.addWeighted(overlay, 0.6, vis, 0.4, 0)
        if human_px is not None:
            cv2.circle(vis, human_px, 8, (255, 255, 255), 2)
        if hunter_px is not None:
            cv2.circle(vis, hunter_px, 10, (0, 255, 0), 2)
        cv2.imshow("Thermal Detection", vis)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ThermalDepthAuto()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()