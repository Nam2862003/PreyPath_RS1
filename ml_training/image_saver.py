#!/usr/bin/env python3
import rclpy, cv2, os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Saver(Node):
    def __init__(self):
        super().__init__('img_saver')
        self.sub = self.create_subscription(Image, '/robot1/camera/image', self.cb, 10)
        self.bridge = CvBridge()
        self.k = 0
        self.skip = 15  # save ~every 15th frame
        os.makedirs('sim/images/raw', exist_ok=True)

    def cb(self, msg):
        self.k += 1
        if self.k % self.skip: return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(f'sim/images/raw/frame_{msg.header.stamp.sec}_{self.k}.jpg', img)

def main():
    rclpy.init(); n=Saver(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
