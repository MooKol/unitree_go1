#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_legged_msgs.msg import IMU

class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(IMU, 'imu_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = IMU()
        msg.quaternion = [0.0, 0.0, 0.0, 1.0]
        msg.gyroscope = [0.0, 0.0, 0.0]
        msg.accelerometer = [0.0, 0.0, 0.0]
        msg.rpy = [0.0, 0.0, 0.0]
        msg.temperature = 25
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing IMU data: {msg}')

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
