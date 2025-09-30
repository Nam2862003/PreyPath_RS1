import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Heartbeat(Node):
    def __init__(self):
        super().__init__('preypath_sensors_heartbeat')
        self.pub = self.create_publisher(String, '/preypath/sensors/heartbeat', 10)
        self.timer = self.create_timer(1.0, self.tick)
        self.count = 0

    def tick(self):
        msg = String()
        msg.data = f'sensors alive {self.count}'
        self.pub.publish(msg)
        self.count += 1

def main():
    rclpy.init()
    node = Heartbeat()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

