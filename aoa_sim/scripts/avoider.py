#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

def min_range(scan: LaserScan):
    vals = [r for r in scan.ranges if not math.isinf(r) and not math.isnan(r)]
    return min(vals) if vals else float('inf')

class Avoider(Node):
    def __init__(self):
        super().__init__('avoider')
        qos = 10
        self.sub_front = self.create_subscription(LaserScan, '/aoa/front/scan', self.cb_front, qos)
        self.sub_left  = self.create_subscription(LaserScan, '/aoa/left/scan',  self.cb_left,  qos)
        self.sub_right = self.create_subscription(LaserScan, '/aoa/right/scan', self.cb_right, qos)
        self.pub_cmd   = self.create_publisher(Twist, '/aoa/cmd_vel', qos)

        self.front = self.left = self.right = float('inf')
        self.timer = self.create_timer(0.05, self.control)  # 20 Hz
        # قيم افتراضية؛ لاحقًا هنقراها من params.yaml عبر Node.declare_parameter/get_parameter
        self.safe = 0.35        # m
        self.cruise_speed = 0.18
        self.backoff_speed = -0.05
        self.max_angular = 0.9
        self.gain_bias = 0.6

    def cb_front(self, msg): self.front = min_range(msg)
    def cb_left(self,  msg): self.left  = min_range(msg)
    def cb_right(self, msg): self.right = min_range(msg)

    def control(self):
        cmd = Twist()
        if self.front < self.safe:
            cmd.linear.x = self.backoff_speed
            cmd.angular.z = self.max_angular if self.left > self.right else -self.max_angular
        else:
            cmd.linear.x = self.cruise_speed
            bias = (self.left - self.right)
            cmd.angular.z = max(min(bias * self.gain_bias, self.max_angular), -self.max_angular)
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    node = Avoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
