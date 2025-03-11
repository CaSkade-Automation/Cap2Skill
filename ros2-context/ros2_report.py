import rclpy
from ros2topic.api import get_topic_names_and_types
from ros2service.api import get_service_names_and_types
import subprocess
import datetime
import time
import logging
import json
from enum import Enum

class Communication(Enum):
    TOPICS = "topics"
    SERVICES = "services"
    ACTIONS = "actions"

COMMUNICATION_FUNCTIONS ={
    Communication.TOPICS: get_topic_names_and_types,
    Communication.SERVICES: get_service_names_and_types,
} 

# Initialize logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger("ros2_inspector")

def generate_report() -> None:
    rclpy.init()
    
    node = rclpy.create_node("ros2_inspector")
    logger.info("ROS2 inspection started.")

    # Header
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    report = {"timestamp": timestamp, "ros2_control_entities": {}}  
    
    for comm_type in Communication:
        comm_list = get_stable_communications(node, comm_type)
        report["ros2_entities"][comm_type.value] = write_interface_details(comm_list, comm_type.value[:-1])

    # Save in file
    file_name = "ros2_system_report.json"
    with open(file_name, "w") as file:
        json.dump(report, file, indent=4)
    
    logger.info(f"Report was saved: {file_name}")
    
    node.destroy_node()
    rclpy.shutdown()

    logger.info("ROS2 inspection finished.")

def get_stable_communications(node, communication_type: Communication, retries=5, delay=1.0) -> list[tuple[str,list[str]]] | list[tuple[str, str]]: 
    """Retries interface discovery until the results stabilize."""
    prev_result = [] 

    get_communications = COMMUNICATION_FUNCTIONS.get(communication_type)

    if get_communications is None: 
        logger.error(f"Unknown communication type:{communication_type} ")
        return []  

    for _ in range(retries):
        communications = get_communications(node=node)
        
        logger.info(f"Attempt {_+1} - Number of {communication_type.value}: " + str(len(communications)))
        
        if not communications:
            logger.warning(f"No active {communication_type.value} found (Attempt {_+1}")

        if communications == prev_result:  # If the results stabilize, return them
            return communications
        
        prev_result = communications.copy()
        time.sleep(delay)
    return prev_result # Return last known stable state

def write_interface_details(communication_list: list[tuple[str,list[str]]] | list[tuple[str, str]], comm_type: str) -> str:
    """Append communication details (interface) to the report."""
    interface_details = []
    for communication_name, interfaces in communication_list:
        ros2_entity = {
            "name": communication_name,
            "type": comm_type, 
            "interfaces": [
                {
                    "name": interface, 
                    "details": get_interface_details(interface) 
                } for interface in (interfaces if isinstance(interfaces, list) else [interfaces])
            ]
        }
        interface_details.append(ros2_entity)
    return interface_details

def get_interface_details(interface: str) -> str:
    """Retrieve detailed information for a given ROS2 interface type."""
    try:
        result = subprocess.run(["ros2", "interface", "show", interface], capture_output=True, text=True)
        return result.stdout.strip() if result else logger.error(f"Error when retrieving the type {interface}:{result.stderr.strip()}")
    except subprocess.CalledProcessError as e:
        logger.error(f"Error retrieving type {interface}: {e.stderr.strip() if e.stderr else str(e)}")
    except Exception as e:
        logger.error(f"Error when retrieving the type: {str(e)}", exc_info=True)
    
# not implemented from ros2 api: actions = get_action_names_and_types(node=node).     
def get_action_names_and_types_cli(node) -> list[tuple[str, str] ] :
    try:
        result = subprocess.run(["ros2", "action", "list", "-t"], capture_output=True, text=True)
        if not result: 
            return[] 
        actions = [] 
        for line in result.stdout.strip().split("\n"):
            parts = line.split(" ")
            if len(parts) == 2:
                actions.append((parts[0], parts[1].lstrip("[").rstrip("]")))
        return actions
    except subprocess.CalledProcessError as e:
        logger.error(f"Error retrieving ros2 actions:{str(e)}") 
    except Exception as e:
        logger.error(f"Unexpected error retrieving ros2 actions: {str(e)}", exc_info=True)
    
    return [] 
    
COMMUNICATION_FUNCTIONS[Communication.ACTIONS] = get_action_names_and_types_cli 

if __name__ == "__main__":
    generate_report()
