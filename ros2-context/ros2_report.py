import rclpy
from ros2topic.api import get_topic_names_and_types
from ros2service.api import get_service_names_and_types
import subprocess
import datetime
import time
import logging

# Initialize logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger("ros2_inspector")

def generate_report():
    rclpy.init()
    
    node = rclpy.create_node("ros2_inspector")
    logger.info("ROS2 inspection started.")

    # Header
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    report = f"# ROS2 System Report created on: {timestamp}"  
    
    report += "\n\n## Topics\n"
    # Topics
    topics = get_stable_topics(node=node)
    if not topics:
        logger.warning("No active topics found")
    report = write_communication_details(topics, report)
    
    report += "\n## Services\n"
    # Services
    services = get_stable_services(node=node)
    if not services:
        logger.warning("No active services found")
    report = write_communication_details(services, report)
    
    report += "\n## Actions\n"
    # Actions
    actions = get_action_names_and_types_cli()
    if not actions:
        logger.warning("No active actions found")
    logger.info("Number of actions: " + str(len(actions)))
    for action_name, action_type in actions:
        report += f"- **{action_name}**: (action_types)\n"
        action_info = get_interface_details(action_type)
        report += f"  - **Message Type Details:**\n{action_info}\n"


    # Save in file
    file_name = "ros2_system_report.md"
    with open(file_name, "w") as file:
        file.write(report)
    
    logger.info(f"Report was saved: {file_name}")
    
    node.destroy_node()
    rclpy.shutdown()

    logger.info("ROS2 inspection finished.")

def get_stable_topics(node, retries=5, delay=1.0):
    """Retries topic discovery until the results stabilize."""
    prev_result =[] 
    for _ in range(retries):
        topics = get_topic_names_and_types(node=node)
        logger.info("Number of topics: " + str(len(topics)))
        if topics == prev_result:  # If the results stabilize, return them
            return topics
        prev_result = topics
        time.sleep(delay)
    return prev_result  # Return last known stable state

def get_stable_services(node, retries=5, delay=1.0):
    """Retries topic discovery until the results stabilize."""
    prev_result =[] 
    for _ in range(retries):
        topics = get_service_names_and_types(node=node)
        logger.info("Number of services: " + str(len(topics)))
        if topics == prev_result:  # If the results stabilize, return them
            return topics
        prev_result = topics
        time.sleep(delay)
    return prev_result  # Return last known stable state

def write_communication_details(communication_list, report):
    for communication_name, communication_types in communication_list:
        report += f"- **{communication_name}**:\n  - Type: {', '.join(communication_types)}\n"
        for communication_type in communication_types:
            communication_info = get_interface_details(communication_type)
            report += f"  - **Type Details:**\n{communication_info}\n"
    return report

def get_interface_details(interface_type):
    try:
        result = subprocess.run(["ros2", "interface", "show", interface_type], capture_output=True, text=True)
        if result.returncode == 0:
            return result.stdout.strip()
        else:
            logger.error(f"Error when retrieving the type {interface_type}: {result.stderr.strip()}")
            return f"Error when retrieving the type: {result.stderr.strip()}"
    except Exception as e:
        logger.error(f"Error when retrieving the type: {str(e)}", exc_info=True)
        return f"Error when retrieving the type: {str(e)}"

# not implemented from ros2 api: actions = get_action_names_and_types(node=node).     
def get_action_names_and_types_cli():
    try:
        result = subprocess.run(["ros2", "action", "list", "-t"], capture_output=True, text=True)
        if result.returncode == 0:
            actions = [] 
            for line in result.stdout.strip().split("\n"):
                parts = line.split(" ")
                if len(parts) == 2:
                    actions.append((parts[0], parts[1].lstrip("[").rstrip("]")))
            return actions
        else:
            logger.error(f"Error when retrieving the actions: {result.stderr.strip()}")
            return [] 
    except Exception as e:
        logger.error(f"Error when retrieving the actions: {str(e)}", exc_info=True)
        return [] 

if __name__ == "__main__":
    generate_report()
