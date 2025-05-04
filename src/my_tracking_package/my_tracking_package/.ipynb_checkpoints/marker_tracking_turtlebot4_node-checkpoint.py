import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        self.bridge = CvBridge()
        self.get_logger().info('ArucoTracker node initialized')
        self.subscription = self.create_subscription(
            Image,
            '/color/preview/image',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_x = 320  # Assuming image size is 640x480
        self.target_y = 240
        self.kp_linear = 0.001  # Proportional gain for linear velocity
        self.kp_angular = 0.001  # Proportional gain for angular velocity
        self.linear_vel_max = 0.2  # Maximum linear velocity
        self.angular_vel_max = 0.2  # Maximum angular velocity
        self.min_distance = 50  # Minimum distance in pixels to consider the marker close
        self.marker_size = 0.1  # Size of the Aruco marker in meters
        self.focal_length = 554  # Focal length of the camera in pixels (adjust according to your camera)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_image = cv2.imread('/home/yoon/practice/marker.png', cv2.IMREAD_GRAYSCALE)
        self.marker_id = 10  # Assuming the ID of the aruco marker is 10
        self.prev_error_x = 0
        self.prev_error_y = 0

    def detect_marker(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def draw_marker(self, image, corners, ids):
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

    def image_callback(self, msg):
        self.get_logger().info('Image callback executed')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect_marker(cv_image)

        # Draw detected markers on the image
        self.draw_marker(cv_image, corners, ids)

        if ids is not None:
            self.get_logger().info('Aruco marker detected!')
            for i in range(len(ids)):
                if ids[i] == self.marker_id:
                    # Match the detected Aruco marker with the template marker
                    res = cv2.matchTemplate(gray, self.marker_image, cv2.TM_CCOEFF)
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                    top_left = max_loc
                    bottom_right = (top_left[0] + self.marker_image.shape[1], top_left[1] + self.marker_image.shape[0])
                    # Draw rectangle around the detected marker
                    cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)

                    cx = int((top_left[0] + bottom_right[0]) / 2)
                    cy = int((top_left[1] + bottom_right[1]) / 2)
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                    error_x = self.target_x - cx
                    error_y = self.target_y - cy
                    twist_msg = self.calculate_velocity(error_x, error_y)
                    self.publisher.publish(twist_msg)
                    break

        cv2.imshow('Aruco Tracker', cv_image)
        cv2.waitKey(1)

    def calculate_velocity(self, error_x, error_y):
        self.get_logger().info('Calculate velocity callback executed')
        # Proportional control
        linear_vel = max(min(self.kp_linear * error_y, self.linear_vel_max), -self.linear_vel_max)
        angular_vel = max(min(self.kp_angular * error_x, self.angular_vel_max), -self.angular_vel_max)

        # Derivative control (optional)
        d_error_x = error_x - self.prev_error_x
        d_error_y = error_y - self.prev_error_y
        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # Publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        return twist_msg

def main(args=None):
    rclpy.init(args=args)
    aruco_tracker = ArucoTracker()
    rclpy.spin(aruco_tracker)
    aruco_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
