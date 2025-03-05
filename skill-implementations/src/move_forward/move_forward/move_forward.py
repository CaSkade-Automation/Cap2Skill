import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveForwardSkill(Node): 

    def __init__(self):
        super().__init__('move_forward')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.velocity_cmd = Twist()

    def move_forward(self, distance: float, duration: float) -> None: 
        velocity = distance / duration
        self.velocity_cmd.linear.x = velocity

        start_time = time.time()

        self.velocity_publisher.publish(self.velocity_cmd)
        self.get_logger().info(f'Publishing velocity: {self.velocity_cmd.linear.x}')

        time.sleep(duration)

        self.velocity_cmd.linear.x = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)
        self.get_logger().info(f'Movement finished.')

        end_time = time.time()
        actual_time = end_time - start_time

        # TODO: better use of robot position to determine traveled distance
        actual_distance = velocity * actual_time

        self.get_logger().info(f"Distance traveled: {actual_distance:.2f}m")
        self.get_logger().info(f"Time needed: {actual_time:.2f}s")


def main(args=None):
    rclpy.init(args=args)

    move_forward_skill = MoveForwardSkill()

    move_forward_skill.move_forward(2, 4)

    rclpy.spin(move_forward_skill)

    move_forward_skill.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()