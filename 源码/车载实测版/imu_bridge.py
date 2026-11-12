import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class Bridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        self.pub = self.create_publisher(Imu, '/imu', 10)
        self.create_subscription(Imu, '/IMU_data', self.cb, 10)

    def cb(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    n = Bridge()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
