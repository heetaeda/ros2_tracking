#!/usr/bin/env python

import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class ObjectTrackingController(Node):
    def __init__(self):
        super().__init__('object_tracking_controller')
        self.subscription = self.create_subscription(Point, '/red_object/center', self.center_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_center = Point()
        self.target_center.x = 320  # 빨간색 사각형이 카메라 이미지의 중심에 있을 때의 x 좌표값

    def center_callback(self, msg):
        self.target_center = msg
        self.move_turtlebot()

    def move_turtlebot(self):
        twist_msg = Twist()
        # 빨간색 사각형이 중앙에 위치하도록 로봇을 조향
        if self.target_center.x < 300:
            twist_msg.angular.z = 0.5  # 왼쪽으로 회전
        elif self.target_center.x > 340:
            twist_msg.angular.z = -0.5  # 오른쪽으로 회전
        else:
            twist_msg.linear.x = 0.2  # 전진
        self.cmd_vel_pub.publish(twist_msg)

# test change

def main(args=None):
    rp.init(args=args)
    controller = ObjectTrackingController()
    rp.spin(controller)
    rp.shutdown()

if __name__ == '__main__':
    main()
