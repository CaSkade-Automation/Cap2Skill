import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SetVelocitySkill(Node): 

    def __init__(self):
        super().__init__('set_velocity')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.velocity_cmd = Twist()

    def publish_velocity(self, velocity: float) -> None: 
        self.velocity_cmd.linear.x = velocity
        self.velocity_publisher.publish(self.velocity_cmd)
        self.get_logger().info(f'Publishing velocity: {self.velocity_cmd.linear.x}')


def main(args=None):
    rclpy.init(args=args)

    set_velocity_skill = SetVelocitySkill()

    set_velocity_skill.publish_velocity(3.0)

    rclpy.spin(set_velocity_skill)

    set_velocity_skill.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()