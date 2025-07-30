#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymycobot.mycobot import MyCobot

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        # MyCobot 연결 (포트와 보드레이트는 실제 환경에 맞게 설정)
        self.mc = MyCobot('/dev/ttyJETCOBOT', 1000000)
        self.get_logger().info("✅ MyCobot Pick and Place Node Started")

        # AprilTag Pose 구독
        self.subscription = self.create_subscription(
            PoseStamped,
            '/april_tag_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        """AprilTag Pose를 받아 MyCobot 위치 제어"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.get_logger().info(f"🔹 이동 명령: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        # MyCobot은 mm 단위 사용 -> m → mm 변환
        coords = [x*1000, y*1000, z*1000, 0, 90, 0]
        self.mc.send_coords(coords, 20, 1)

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
