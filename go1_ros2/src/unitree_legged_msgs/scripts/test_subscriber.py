#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_legged_msgs.msg import IMU

class IMUSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(IMU, 'imu_topic', self.listener_callback, 10)
        self.subscription # Unused var warn

    def listener_callback(self, msg):
        self.get_logger().info(f'Received IMU data: quaternion={msg.quaternion}, gyroscope={msg.gyroscope}')

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = IMUSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
