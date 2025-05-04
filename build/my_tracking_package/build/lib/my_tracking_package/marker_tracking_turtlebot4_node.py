import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import tf2_ros

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        self.get_logger().info("Aruco Tracker start!!!")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(  # 카메라 이미지 영상 토픽 구독
            Image,
            '/color/preview/image',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_x = 300  # 이미지 크기가 600x400 일 때
        self.target_y = 200
        self.kp_angular = 0.001  # 각속도 제어를 위한 비례 이득 값
        self.angular_vel_max = 0.2  # 최대 각속도 값
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) # 6x6 크기, 250개의 ID를 가진 사전 사용
        self.parameters = aruco.DetectorParameters()  # aruco 마커 감지 매개변수
        self.marker_image = cv2.imread('/home/yoon/practice/aruco_marker_id_10.png', cv2.IMREAD_GRAYSCALE)
        self.marker_id = 10  # 추적할 마커 id
        self.current_angle = 0.0  # 현재 각도 초기화
        self.tf_buffer = tf2_ros.Buffer()  
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def detect_marker(self, image):  # 마커 인식 메서드
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def calculate_marker_side_lengths(self, corners):  # 인식된 마커 네 변의 길이를 계산하는 메서드
        side_lengths = []
        for corner_set in corners:
            side_lengths_per_marker = []
            for i in range(len(corner_set)):
                side_length = np.linalg.norm(corner_set[i] - corner_set[(i + 1) % 4])
                side_lengths_per_marker.append(side_length)
            side_lengths.append(side_lengths_per_marker)
        return side_lengths

    def image_callback(self, msg):  # 이미지창에 영상을 띄우는 메서드
        cv_image_og = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image = cv2.resize(cv_image_og, (600, 400))  # 카메라 영상 크기 조정
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect_marker(cv_image)

        if ids is not None:
            self.get_logger().info('Aruco marker detected!')
            for i in range(len(ids)):
                # 각 마커에 대해 처리
                if ids[i] == self.marker_id:
                    # 마커의 코너 좌표 추출
                    marker_corners = corners[i][0]

                    # 마커의 각 변의 길이 계산
                    side_lengths = self.calculate_marker_side_lengths([marker_corners])

                    # 마커의 네 변 길이의 합
                    perimeter = sum(side_lengths[0])

                    # 인식된 마커의 테두리 그리기
                    cv2.aruco.drawDetectedMarkers(cv_image, corners)

                    # 마커의 중심 계산
                    cx = int(np.mean(marker_corners[:, 0]))
                    cy = int(np.mean(marker_corners[:, 1]))

                    # 빨간색 점 그리기
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

                    # 마커의 ID를 표시할 위치 계산
                    text_position = (int(marker_corners[0][0]), int(marker_corners[0][1]) - 10)

                    # 텍스트 추가
                    cv2.putText(cv_image, f'ID: {ids[i]}', text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    
                    # 마커 네 변의 길이 합 표시
                    cv2.putText(cv_image, f'Perimeter: {perimeter:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                    # 마커의 각 변의 길이 표시
                    for j, length in enumerate(side_lengths[i]):
                        cv2.putText(cv_image, f'Side {j+1}: {length:.2f}', (text_position[0], text_position[1] - 20 - 20 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    # 이미지의 중심 좌표 계산
                    image_center_x = cv_image.shape[1] // 2  # 카메라 영상 화면 중심의 x 좌표
                    image_center_y = cv_image.shape[0] // 2  # 카메라 영상 화면 중심의 y 좌표

                    # 파란색 점 그리기
                    cv2.circle(cv_image, (image_center_x, image_center_y), 5, (255, 0, 0), -1)
                   
                    # 마커 중앙 좌표와 이미지 중앙 좌표 비교
                    if ((cx > image_center_x) or (cx < image_center_x)) and (abs(cx - image_center_x) > 50.0):  # aruco 마커가 화면에 인식되고, 마커 중심과 화면 중심 사이 픽셀 좌표 x 값이 50 보다 크면
                        self.rotate_turtle("try to track", cx, image_center_x, side_lengths)
                            
                    elif abs(cx - image_center_x) <= 50.0:  # 임계값 안에 들어왔을 때
                        self.rotate_turtle("try to stop", cx, image_center_x, side_lengths)                                                                          

        cv2.imshow('Aruco Tracker', cv_image)
        key = cv2.waitKey(1)
        if key == 27:  # ESC 키
            cv2.destroyAllWindows()
            self.get_logger().info('ESC 키가 눌려 이미지 창을 종료합니다.')
            rclpy.shutdown()

    def rotate_turtle(self, order, cx, image_center_x, side_lengths):  # 터틀봇을 구동시키는 메서드
        twist_msg = Twist()  # Twist() 토픽의 angular.z, linear.x 메세지 타입 사용
        if order == "try to track":  
            self.get_logger().info("try to track!!!")
            twist_msg.linear.x = 0.15
            if cx > image_center_x:
                twist_msg.angular.z = -0.15
            elif cx < image_center_x:
                twist_msg.angular.z = 0.15
                
        else:                          # order = try to stop
            if sum(side_lengths[0]) < 300:
                self.get_logger().info("try to stop!!!")
                twist_msg.linear.x = 0.08
                if (cx > image_center_x) and sum(side_lengths[0]) < 450.:
                    twist_msg.angular.z = -0.08
                elif (cx < image_center_x) and sum(side_lengths[0]) < 450.:
                    twist_msg.angular.z = 0.08

            else:                     # sum(side_lengths[0]) >= 300
                twist_msg.linear.x = 0.04
                if (cx > image_center_x) and sum(side_lengths[0]) < 450.:
                    self.get_logger().info("try to stop!!!")
                    twist_msg.angular.z = -0.04
                elif (cx < image_center_x) and sum(side_lengths[0]) < 450.:
                    self.get_logger().info("try to stop!!!")
                    twist_msg.angular.z = 0.04
                elif sum(side_lengths[0]) >= 450.:
                    self.get_logger().info("arrive successfully")
                    twist_msg.angular.z = 0.0
                    twist_msg.linear.x = 0.0
                    
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    aruco_tracker = ArucoTracker()
    rclpy.spin(aruco_tracker)
    aruco_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()