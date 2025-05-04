#!/usr/bin/env python

import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image  # oak-D 카메라에서 나오는 영상 토픽 
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.center_pub = self.create_publisher(Point, '/red_object/center', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        # 이미지를 HSV 색 공간으로 변환
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 빨간색의 HSV 범위 설정
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        # HSV 이미지에서 빨간색 객체를 찾기 위한 마스크 생성
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # 마스크를 사용하여 이미지에서 빨간색 객체를 추출
        red_object_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # 빨간색 객체의 외곽선 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 빨간색 객체의 외곽선 중심점 찾기
        for contour in contours:
            # 빨간색 사각형의 최소 크기 설정
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                # 사각형 중심점 계산
                center_x = x + w // 2
                center_y = y + h // 2
                # 추적된 빨간색 사각형의 중심점 출력
                print("빨간색 사각형 중심점:", center_x, center_y)
                # 빨간색 사각형의 중심점을 Point 메시지로 변환하여 게시
                center_msg = Point()
                center_msg.x = center_x
                center_msg.y = center_y
                center_msg.z = 0
                self.center_pub.publish(center_msg)

        # 결과 이미지 표시
        cv2.imshow("Red Object Tracking", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rp.init(args=args)
    node = ObjectTrackingNode()
    rp.spin(node)
    rp.shutdown()

if __name__ == '__main__':
    main()
