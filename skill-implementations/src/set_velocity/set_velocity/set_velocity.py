import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SetVelocitySkill(Node): 

    MAX_VELOCITY = 0.8

    def __init__(self):
        super().__init__('set_velocity')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.velocity_cmd = Twist()

    def publish_velocity(self, velocity: float) -> None: 
        if velocity > self.MAX_VELOCITY:
            self.get_logger().warn(f"Desired velocity {velocity} exceeds the maximum of {self.MAX_VELOCITY}. Limitation is applied.")
            velocity = self.MAX_VELOCITY

        self.velocity_cmd.linear.x = velocity
        self.velocity_publisher.publish(self.velocity_cmd)
        self.get_logger().info(f'Publishing velocity: {self.velocity_cmd.linear.x}')


def main(args=None):
    rclpy.init(args=args)

    set_velocity_skill = SetVelocitySkill()

    set_velocity_skill.publish_velocity(0.5)

    rclpy.spin(set_velocity_skill)

    set_velocity_skill.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()