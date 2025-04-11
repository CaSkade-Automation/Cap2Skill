import time
import random
from pyskillup.decorators.decorators import *
from pyskillup.decorator_check.decorator_check import ROS2Skill
from pyskillup.decorators.skill_interface import SkillInterface
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

@skill(skill_interface=SkillInterface.REST, 
       skill_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/skills/mapping/mapping", 
       module_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/neobotixMMO700", 
       capability_iri="http://www.hsu-hh.de/aut/ontologies/mobile-robots/neobotix/mmo700/capabilities/mapping/mapping", 
       description="Map an area of a desired length x and y based on the mobile robot's position by moving the mobile robot in that area.")
class MappingSkill(ROS2Skill):

    def __init__(self):
        super().__init__("mapping")

        self.velocity_publisher = None
        self.map_service_client = None
        self.map_data = None
        self.mapping_complete = False

    @skill_parameter(is_required=True, name="area_length_x", description="The length of the area to map in the x direction.")
    def get_area_length_x(self) -> float:
        return self.area_length_x

    @skill_parameter(is_required=True, name="area_length_y", description="The length of the area to map in the y direction.")
    def get_area_length_y(self) -> float:
        return self.area_length_y

    @skill_output(is_required=True, name="generated_map", description="The generated map of the area.")
    def get_generated_map(self) -> OccupancyGrid:
        return self.map_data

    @starting
    def starting(self) -> None:
        self.node.get_logger().info("MappingSkill is starting with creating a publisher to cmd_vel topic and a service client for map retrieval")
        self.velocity_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.map_service_client = self.node.create_client(GetMap, '/slam_toolbox/dynamic_map')

    @execute
    def execute(self) -> None:
        self.node.get_logger().info("MappingSkill is executing, moving the robot to map the area")
        twist = Twist()
        start_time = time.time()
        mapping_duration = 60  # seconds

        while time.time() - start_time < mapping_duration:
            twist.linear.x = random.uniform(-0.5, 0.5)
            twist.angular.z = random.uniform(-1.0, 1.0)
            self.velocity_publisher.publish(twist)
            self.node.get_logger().info(f"Moving with linear velocity: {twist.linear.x}, angular velocity: {twist.angular.z}")
            time.sleep(1)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)
        self.node.get_logger().info("Mapping complete, stopping the robot")

        self.node.get_logger().info("Retrieving the generated map")
        if self.map_service_client.wait_for_service(timeout_sec=10.0):
            future = self.map_service_client.call_async(GetMap.Request())
            future.add_done_callback(self.map_response_callback)
        else:
            self.node.get_logger().error("Map service not available")

    def map_response_callback(self, future):
        try:
            response = future.result()
            self.map_data = response.map
            self.mapping_complete = True
            self.node.get_logger().info("Map retrieved successfully")
        except Exception as e:
            self.node.get_logger().error(f"Failed to retrieve map: {e}")

    @stopping
    def stopping(self) -> None:
        self.node.get_logger().info("MappingSkill is stopping, setting velocity to zero")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)

    @completing
    def completing(self) -> None:
        if self.mapping_complete:
            self.node.get_logger().info("MappingSkill is completing, map generated successfully")
        else:
            self.node.get_logger().info("MappingSkill is completing, but map generation was not successful")

    @resetting
    def resetting(self) -> None:
        self.node.get_logger().info("MappingSkill is resetting")
        self.velocity_publisher.destroy()
        self.map_service_client.destroy()
        self.mapping_complete = False