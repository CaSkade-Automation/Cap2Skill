import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class GetPositionSkill(Node): 

    def __init__(self):
        super().__init__('get_position')
        self.odom_subscription = self.create_subscription(Odometry, "odom", self.odom_listener_callback, 10)

    def odom_listener_callback(self, msg: Odometry) -> None: 
        position = Point()
        position = msg.pose.pose.position
        self.get_logger().info(f'Position: x={position.x}, y={position.y}')

def main(args=None):
    rclpy.init(args=args)

    get_position_skill = GetPositionSkill()

    rclpy.spin(get_position_skill)

    get_position_skill.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()