import rclpy
import threading
from rclpy.executors import SingleThreadedExecutor
# add generated imports (but not pyskillup imports)

class TestSkill(): 

    def __init__(self, node_name: str):
        self._ros_initialized = False
        self._executor = None
        self._executor_thread = None
        self.node = None
        self.node_name = node_name
        self._ensure_ros()

        # Add generated init attributes

    def _ensure_ros(self):
        if not self._ros_initialized:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node(self.node_name)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self.node)
            self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._executor_thread.start()
            self._ros_initialized = True

    def shutdown_ros(self):
        if self._ros_initialized:
            self._executor.shutdown()
            self.node.destroy_node()
            self._ros_initialized = False

    # Add skill parameters 
    
    # add skill outputs
    
    # add skill behavior in states


def main(args=None):
    test_skill = TestSkill("test_skill")

    # set parameters if needed 

    # change methods to test different skill states
    test_skill.starting()
    test_skill.execute()
    test_skill.completing()
    # set_velocity.stopping()
    # test_skill.resetting()

if __name__ == '__main__':
    main()