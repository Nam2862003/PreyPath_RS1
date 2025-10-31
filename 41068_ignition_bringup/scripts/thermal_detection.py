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

        # Publisher for human pose
        self.pose_pub = self.create_publisher(PointStamped, '/human_pose', 10)

        # Synchronizer for thermal + depth
        self.ts = ApproximateTimeSynchronizer(
            [self.thermal_sub, self.depth_sub, self.thermal_info, self.depth_info],
            queue_size=10,
            slop=0.2
        )
        self.ts.registerCallback(self.callback)

        # Store first detected world position
        self.human_fixed = None

        self.get_logger().info("Thermal-Depth auto node ready.")

    def callback(self, thermal_msg, depth_msg, tinfo, dinfo):
        # Convert image to numpy
        thermal = self.bridge.imgmsg_to_cv2(thermal_msg, 'mono16').astype(np.float32) / 100.0
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)

        # Detect hot human-like region
        human_mask = (thermal >= 308.0) & (thermal < 310.5)
        if not np.any(human_mask):
            return

        # Get centroid pixel of detected human
        y, x = np.mean(np.column_stack(np.where(human_mask)), axis=0)
        u_d = int(np.clip(x, 0, depth.shape[1] - 1))
        v_d = int(np.clip(y, 0, depth.shape[0] - 1))

        depth_value = float(depth[v_d, u_d])
        if np.isnan(depth_value) or depth_value <= 0.05:
            self.get_logger().warn("Invalid depth, skipping.")
            return

        # Camera intrinsics
        fx, fy, cx, cy = dinfo.k[0], dinfo.k[4], dinfo.k[2], dinfo.k[5]

        # Create 3D point in camera frame
        X_opt = (u_d - cx) * depth_value / fx
        Y_opt = (v_d - cy) * depth_value / fy
        Z_opt = depth_value

        pt_cam = PointStamped()
        pt_cam.header.frame_id = dinfo.header.frame_id  # typically camera_depth_optical_frame
        pt_cam.header.stamp = self.get_clock().now().to_msg()
        pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = X_opt, Y_opt, Z_opt

        try:
            # Transform to map frame (world coordinate)
            tf = self.tf_buffer.lookup_transform('map', pt_cam.header.frame_id, rclpy.time.Time())
            pt_world = do_transform_point(pt_cam, tf)

            # Save the first detected human position (fixed)
            if self.human_fixed is None:
                self.human_fixed = (pt_world.point.z, pt_world.point.y, pt_world.point.x)
                self.get_logger().info(
                    f"📍 Fixed human position saved at "
                    f"({self.human_fixed[0]:.2f}, {self.human_fixed[1]:.2f}, {self.human_fixed[2]:.2f})"
                )

            # Publish the fixed human pose (constant world position)
            msg = PointStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = self.human_fixed[0]
            msg.point.y = self.human_fixed[1]
            msg.point.z = 0.0  # keep it flat on map (2D)
            self.pose_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"TF fail: {e}")
            return

        # Visualization overlay (for debug)
        vis = cv2.applyColorMap(
            np.uint8(np.clip((thermal - 309.15) * 2, 0, 255)), cv2.COLORMAP_JET
        )
        overlay = vis.copy()
        overlay[human_mask] = [0, 0, 255]  # red overlay
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
