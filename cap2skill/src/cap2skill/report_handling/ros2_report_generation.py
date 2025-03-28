import subprocess
import datetime
import time
import logging
from enum import Enum

class Communication(Enum):
    TOPICS = "topics"
    SERVICES = "services"
    ACTIONS = "actions"

COMMUNICATION_FUNCTIONS ={}

# Initialize logging
logger = logging.getLogger("ros2_inspector")

def generate_report() -> str:
    """
    Generates a report of ROS2 communication entities including topics, services, and actions.
    This function initializes a ROS2 node, retrieves the available communication entities
    (topics, services, and actions), and generates a structured report containing their details.
    The report includes a timestamp and categorized information about the ROS2 entities.
    Returns:
        str: A JSON string representation of the report containing ROS2 communication entities.
    Raises:
        ImportError: If the required ROS2 packages are not installed or sourced properly.
    """
    try: 
        import rclpy
        from ros2topic.api import get_topic_names_and_types
        from ros2service.api import get_service_names_and_types
    except ImportError as e:
        logger.error(f"ROS2 packages not found. Make sure ROS2 is installed and sourced: {str(e)}")
        raise e
    
    COMMUNICATION_FUNCTIONS.update({
        Communication.TOPICS: get_topic_names_and_types,
        Communication.SERVICES: get_service_names_and_types,
        Communication.ACTIONS: get_action_names_and_types_cli
    })

    rclpy.init()
    
    node = rclpy.create_node("ros2_inspector")
    logger.info("ROS2 inspection node started.")

    # Header
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    report = {"timestamp": timestamp, "ros2_control_entities": {}}  
    
    report["ros2_control_entities"] =[] 

    for comm_type in Communication:
        comm_list = get_stable_communications(node, comm_type)
        report["ros2_control_entities"].extend(write_interface_details(comm_list, comm_type.value[:-1]))
    
    node.destroy_node()
    rclpy.shutdown()

    logger.info("ROS2 inspection finished.")
    return report

def get_stable_communications(node, communication_type: Communication, retries=5, delay=1.0) -> list[tuple[str,list[str]]] | list[tuple[str, str]]: 
    """
    Retries interface discovery for a specified communication type until the results stabilize 
    or the maximum number of retries is reached.
    Args:
        node: The ROS2 node instance used to query communications.
        communication_type (Communication): The type of communication to discover (i.e., topic, service or action).
        retries (int, optional): The maximum number of attempts to stabilize the results. Defaults to 5.
        delay (float, optional): The delay in seconds between retries. Defaults to 1.0.
    Returns:
        list[tuple[str, list[str]]] | list[tuple[str, str]]: A list of stabilized communication results. 
        Each tuple contains a string identifier and associated data. Returns an empty list if the 
        communication type is unknown or no results are found.
    """
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
    """
    Append communication details (interface) to the report.

    Args:
        communication_list (list[tuple[str, list[str]]] | list[tuple[str, str]]): 
            A list of tuples where each tuple contains a communication name and its associated interfaces. 
            The interfaces can either be a list of strings or a single string.
        comm_type (str): 
            The type of communication (i.e., topic, service, action).

    Returns:
        str: A JSON string representation of the communication details, 
                including the communication name, type, and interface details.

    Notes:
        - The function processes each communication entry, retrieves detailed information 
            for each interface using `get_interface_details`, and logs the addition of the 
            communication name to the system report.
        - The `get_interface_details` function is expected to provide specific details 
            about a given interface.
    """
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
        logger.info(f"Added {communication_name} to the system report.")
    return interface_details

def get_interface_details(interface: str) -> str:
    """
    Retrieve detailed information for a given ROS2 interface type.

    This function executes a subprocess command to fetch details about a specified
    ROS2 interface type using the `ros2 interface show` command. The output of the
    command is returned as a string.

    Args:
        interface (str): The name of the ROS2 interface type to retrieve details for.

    Returns:
        str: The detailed information about the specified ROS2 interface type, or
            logs an error and returns None if an error occurs.

    Raises:
        Logs errors for subprocess.CalledProcessError or other exceptions that may
        occur during the execution of the subprocess command.
    """
    try:
        result = subprocess.run(["ros2", "interface", "show", interface], capture_output=True, text=True)
        return result.stdout.strip() if result else logger.error(f"Error when retrieving the type {interface}:{result.stderr.strip()}")
    except subprocess.CalledProcessError as e:
        logger.error(f"Error retrieving type {interface}: {e.stderr.strip() if e.stderr else str(e)}")
    except Exception as e:
        logger.error(f"Error when retrieving the type: {str(e)}", exc_info=True)
    
# not implemented from ros2 api: actions = get_action_names_and_types(node=node).     
def get_action_names_and_types_cli(node) -> list[tuple[str, str] ] :
    """
    Retrieve a list of ROS 2 action names and their corresponding types.
    This function executes the `ros2 action list -t` command to fetch all available
    ROS 2 actions along with their types. The output is parsed and returned as a list
    of tuples, where each tuple contains the action name and its type.
    Args:
        node: The ROS 2 node instance (not used in the current implementation).
    Returns:
        list[tuple[str, str]]: A list of tuples, where each tuple contains:
            - str: The name of the action.
            - str: The type of the action.
    Exceptions:
        Logs an error message if a `subprocess.CalledProcessError` occurs or if any
        other unexpected exception is raised during the execution of the command.
    Note:
        If the command fails or no actions are found, an empty list is returned.
    """
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