#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
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

        # Detect hot region (human)
        human_mask = (thermal >= 308.0) & (thermal < 310.5)
        if not np.any(human_mask):
            self.get_logger().warn("No human detected.")
            return

        # Get centroid pixel
        y, x = np.mean(np.column_stack(np.where(human_mask)), axis=0)
        u_d, v_d = int(np.clip(x, 0, depth.shape[1] - 1)), int(np.clip(y, 0, depth.shape[0] - 1))
        depth_value = float(depth[v_d, u_d])

        if np.isnan(depth_value) or depth_value <= 0.01:
            self.get_logger().warn("Invalid depth, skipping.")
            return

        # Initialize camera model (only once)
        if self.camera_model is None:
            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.fromCameraInfo(dinfo)
            self.get_logger().info("Camera model initialized from depth_info.")

        # --- Pixel to ray conversion using camera model ---
        ray = self.camera_model.projectPixelTo3dRay((u_d, v_d))
        X_cam = ray[0] * depth_value
        Y_cam = ray[1] * depth_value
        Z_cam = ray[2] * depth_value

        # Create point in camera frame
        pt_cam = PointStamped()
        pt_cam.header.frame_id = dinfo.header.frame_id  # usually "camera_depth_optical_frame"
        pt_cam.header.stamp = self.get_clock().now().to_msg()
        pt_cam.point.x = X_cam
        pt_cam.point.y = Y_cam
        pt_cam.point.z = Z_cam

        try:
            # Transform to map/world frame
            tf = self.tf_buffer.lookup_transform('map', pt_cam.header.frame_id, rclpy.time.Time())
            pt_world = do_transform_point(pt_cam, tf)

            # Publish result
            msg = PointStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = pt_world.point.z
            msg.point.y = pt_world.point.y
            msg.point.z = pt_world.point.x
            self.pose_pub.publish(msg)

            self.get_logger().info(
                f"👤 Human | pixel=({int(x)}, {int(y)}) | depth={depth_value:.2f} m "
                f"| map=({pt_world.point.z:.2f}, {pt_world.point.y:.2f}, {pt_world.point.x:.2f})"
            )

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")

        # --- Visualization (for debugging only) ---
        vis = cv2.applyColorMap(
            np.uint8(np.clip((thermal - 309.15) * 2, 0, 255)), cv2.COLORMAP_JET
        )
        overlay = vis.copy()
        overlay[human_mask] = [0, 0, 255]  # Red overlay
        vis = cv2.addWeighted(overlay, 0.6, vis, 0.4, 0)
        cv2.circle(vis, (int(x), int(y)), 8, (255, 255, 255), 2)
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
