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

        self.thermal_sub = Subscriber(self, Image, '/robot1/thermal/image')
        self.depth_sub = Subscriber(self, Image, '/robot1/camera/depth/image')
        self.thermal_info = Subscriber(self, CameraInfo, '/robot1/thermal/camera_info')
        self.depth_info = Subscriber(self, CameraInfo, '/robot1/camera/camera_info')
        self.pose_pub = self.create_publisher(PointStamped, '/human_pose', 10)

        self.ts = ApproximateTimeSynchronizer(
            [self.thermal_sub, self.depth_sub, self.thermal_info, self.depth_info],
            queue_size=10,
            slop=0.2
        )
        self.ts.registerCallback(self.callback)
        self.get_logger().info("Thermal-Depth auto node ready.")

    def callback(self, thermal_msg, depth_msg, tinfo, dinfo):
        thermal = self.bridge.imgmsg_to_cv2(thermal_msg, 'mono16').astype(np.float32) / 100.0
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)

        human_mask = (thermal >= 308.0) & (thermal < 310.5)
        if not np.any(human_mask):
            self.get_logger().warn("No human detected.")
            return 

        y, x = np.mean(np.column_stack(np.where(human_mask)), axis=0)
        u_d = int(x)
        v_d = int(y)
        u_d = np.clip(u_d, 0, depth.shape[1] - 1)
        v_d = np.clip(v_d, 0, depth.shape[0] - 1)
        depth_value = float(depth[v_d, u_d])
        if np.isnan(depth_value) or depth_value <= 0.01:
            self.get_logger().warn("Invalid depth, skipping.")
            return

        fx, fy, cx, cy = dinfo.k[0], dinfo.k[4], dinfo.k[2], dinfo.k[5]

        pt_cam = PointStamped()
        pt_cam.header.frame_id = dinfo.header.frame_id  # typically "camera_depth_optical_frame"
        pt_cam.header.stamp = self.get_clock().now().to_msg()

        # Convert from optical → camera frame convention
        X_opt = (u_d - cx) * depth_value / fx
        Y_opt = (v_d - cy) * depth_value / fy
        Z_opt = depth_value

        pt_cam.point.x = X_opt
        pt_cam.point.y = Y_opt
        pt_cam.point.z = Z_opt
        
        try:
            tf = self.tf_buffer.lookup_transform('camera_depth_optical_frame', pt_cam.header.frame_id, rclpy.time.Time())
            pt_world = do_transform_point(pt_cam, tf)
            self.get_logger().info(
                f"👤 Human | pixel=({int(x)},{int(y)}) | depth={depth_value:.2f} m "
                f"| world≈({pt_world.point.z:.2f}, {pt_world.point.y:.2f}, {pt_world.point.x:.2f})"
            )
            # Publish human pose
            msg = PointStamped()
            msg.header.frame_id = 'camera_depth_optical_frame'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = pt_world.point.z
            msg.point.y = pt_world.point.y
            msg.point.z = pt_world.point.x
            self.pose_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"TF fail: {e}")

        # Optional visualization
        vis = cv2.applyColorMap(
            np.uint8(np.clip((thermal - 309.15) * 2, 0, 255)), cv2.COLORMAP_JET
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


def main():
    rclpy.init()
    node = ThermalDepthAuto()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
