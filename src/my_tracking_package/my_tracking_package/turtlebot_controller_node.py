import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.get_logger().info('TurtlebotController node initialized!!!!!!!!!')  # 노드 초기화 로그 메시지
        # '/cmd_vel' 토픽을 구독하는 구독자를 생성합니다.
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # 사용하지 않는 변수 경고 방지

    def cmd_vel_callback(self, msg):
        self.get_logger().info('Received Twist message')  # Twist 메시지 수신 로그 메시지
        # Twist 메시지를 받았을 때 실행되는 콜백 함수
        # 여기에 터틀봇을 제어하는 코드를 추가하세요
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        # 받은 linear_velocity와 angular_velocity를 사용하여 터틀봇을 제어하는 코드를 작성하세요

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
