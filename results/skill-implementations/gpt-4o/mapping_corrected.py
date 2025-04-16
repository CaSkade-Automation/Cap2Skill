import time
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
import math

@skill(skill_interface=SkillInterface.REST, 
       skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/mapping/mapping", 
       module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", 
       capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/mapping/mapping", 
       description="Map an area of a desired length x and y based on the mobile robot's position by moving the mobile robot in that area.")
class MappingSkill(ROS2Skill):

    def __init__(self):
        super().__init__("mapping")
        self.velocity_publisher = None
        self.map_subscription = None
        self.map_data = None
        self.received_map = False
        self.velocity_cmd = Twist()
        self.obstacle_detected = False
        self.min_front = float('inf')
        self.get_map_client = None


    @skill_parameter(is_required=True, name="distance_x", description="The desired length of the area to map in the x direction.")
    def get_distance_x(self) -> float:
        return self.distance_x

    @skill_parameter(is_required=True, name="distance_y", description="The desired length of the area to map in the y direction.")
    def get_distance_y(self) -> float:
        return self.distance_y

    @skill_output(is_required=True, name="generated_map", description="The generated map of the area.")
    def get_generated_map(self) -> OccupancyGrid:
        return self.map_data

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("MappingSkill is starting with creating a publisher to cmd_vel topic and subscribing to map topic")
        self.velocity_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.get_map_client = self.node.create_client(GetMap, '/slam_toolbox/dynamic_map')
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for /slam_toolbox/dynamic_map service...")

        self.node.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        num_ranges = len(ranges)

        def angle_to_index(angle_deg):
            angle_rad = angle_deg * math.pi / 180.0
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            return max(0, min(index, num_ranges - 1))

        front_range = range(angle_to_index(-10), angle_to_index(10))
        valid_ranges = [ranges[i] for i in front_range if not (math.isinf(ranges[i]) or math.isnan(ranges[i]))]
        self.min_front = min(valid_ranges) if valid_ranges else float('inf')
        self.obstacle_detected = self.min_front < 0.5

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("MappingSkill is executing, moving the robot to map the area.")
        # Simple square pattern for mapping
        for _ in range(4):
            self.move_straight(self.get_distance_x())
            self.turn_90_degrees()
            self.move_straight(self.get_distance_y())
            self.turn_90_degrees()

        self.node.get_logger().info("Mapping complete, waiting for map data.")
        request = GetMap.Request()
        future = self.get_map_client.call_async(request)

        while not future.done():
            time.sleep(0.1)

        try:
            response = future.result()
            self.map_data = response.map
            self.received_map = True
            self.node.get_logger().info("Received map data from SLAM Toolbox.")
        except Exception as e:
            self.node.get_logger().error(f"Failed to get map: {e}")

    def move_straight(self, distance):
        self.velocity_cmd.linear.x = 0.2  # Move forward with a constant speed
        duration = distance / self.velocity_cmd.linear.x
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.obstacle_detected:
                self.node.get_logger().warn("Obstacle detected – aborting straight move.")
                break
            self.velocity_publisher.publish(self.velocity_cmd)
            time.sleep(0.1)
        self.velocity_cmd.linear.x = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)

    def turn_90_degrees(self):
        self.velocity_cmd.angular.z = 0.5  # Turn with a constant angular speed
        duration = 1.57 / self.velocity_cmd.angular.z  # 90 degrees in radians
        start_time = time.time()
        while time.time() - start_time < duration:
            self.velocity_publisher.publish(self.velocity_cmd)
            time.sleep(0.1)
        self.velocity_cmd.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("MappingSkill is stopping, setting velocity to zero.")
        self.velocity_cmd.linear.x = 0.0
        self.velocity_cmd.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity_cmd)

    @completing
    def completing(self) -> None:
        self.node.get_logger().info("MappingSkill is completing, map data is ready.")

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("MappingSkill is resetting, shutting down processes.")
        self.velocity_publisher.destroy()
        self.received_map = False