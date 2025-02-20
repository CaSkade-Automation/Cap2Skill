# ROS2 System Report created on: 2025-02-18 11:41:25

## Topics
- **/clock**:
  - Type: rosgraph_msgs/msg/Clock
  - **Type Details:**
# This message communicates the current time.
#
# For more information, see https://design.ros2.org/articles/clock_and_time.html.
builtin_interfaces/Time clock
	int32 sec
	uint32 nanosec
- **/cmd_vel**:
  - Type: geometry_msgs/msg/Twist
  - **Type Details:**
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z
- **/dynamic_joint_states**:
  - Type: control_msgs/msg/DynamicJointState
  - **Type Details:**
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# List of resource names, e.g. ["arm_joint_1", "arm_joint_2", "gripper_joint"]
string[] joint_names
# Key-value pairs representing interfaces and their corresponding values for each joint listed in `joint_names`
InterfaceValue[] interface_values
	string[] interface_names
	float64[] values
- **/joint_state_broadcaster/transition_event**:
  - Type: lifecycle_msgs/msg/TransitionEvent
  - **Type Details:**
# The time point at which this event occurred.
uint64 timestamp

# The id and label of this transition event.
Transition transition
	uint8 TRANSITION_CREATE = 0
	uint8 TRANSITION_CONFIGURE = 1
	uint8 TRANSITION_CLEANUP = 2
	uint8 TRANSITION_ACTIVATE = 3
	uint8 TRANSITION_DEACTIVATE = 4
	uint8 TRANSITION_UNCONFIGURED_SHUTDOWN  = 5
	uint8 TRANSITION_INACTIVE_SHUTDOWN = 6
	uint8 TRANSITION_ACTIVE_SHUTDOWN = 7
	uint8 TRANSITION_DESTROY = 8
	uint8 TRANSITION_ON_CONFIGURE_SUCCESS = 10
	uint8 TRANSITION_ON_CONFIGURE_FAILURE = 11
	uint8 TRANSITION_ON_CONFIGURE_ERROR = 12
	uint8 TRANSITION_ON_CLEANUP_SUCCESS = 20
	uint8 TRANSITION_ON_CLEANUP_FAILURE = 21
	uint8 TRANSITION_ON_CLEANUP_ERROR = 22
	uint8 TRANSITION_ON_ACTIVATE_SUCCESS = 30
	uint8 TRANSITION_ON_ACTIVATE_FAILURE = 31
	uint8 TRANSITION_ON_ACTIVATE_ERROR = 32
	uint8 TRANSITION_ON_DEACTIVATE_SUCCESS = 40
	uint8 TRANSITION_ON_DEACTIVATE_FAILURE = 41
	uint8 TRANSITION_ON_DEACTIVATE_ERROR = 42
	uint8 TRANSITION_ON_SHUTDOWN_SUCCESS = 50
	uint8 TRANSITION_ON_SHUTDOWN_FAILURE = 51
	uint8 TRANSITION_ON_SHUTDOWN_ERROR = 52
	uint8 TRANSITION_ON_ERROR_SUCCESS = 60
	uint8 TRANSITION_ON_ERROR_FAILURE = 61
	uint8 TRANSITION_ON_ERROR_ERROR = 62
	uint8 TRANSITION_CALLBACK_SUCCESS = 97
	uint8 TRANSITION_CALLBACK_FAILURE = 98
	uint8 TRANSITION_CALLBACK_ERROR = 99
	##
	##
	uint8 id
	string label

# The starting state from which this event transitioned.
State start_state
	uint8 PRIMARY_STATE_UNKNOWN = 0
	uint8 PRIMARY_STATE_UNCONFIGURED = 1
	uint8 PRIMARY_STATE_INACTIVE = 2
	uint8 PRIMARY_STATE_ACTIVE = 3
	uint8 PRIMARY_STATE_FINALIZED = 4
	uint8 TRANSITION_STATE_CONFIGURING = 10
	uint8 TRANSITION_STATE_CLEANINGUP = 11
	uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
	uint8 TRANSITION_STATE_ACTIVATING = 13
	uint8 TRANSITION_STATE_DEACTIVATING = 14
	uint8 TRANSITION_STATE_ERRORPROCESSING = 15
	uint8 id
	string label

# The end state of this transition event.
State goal_state
	uint8 PRIMARY_STATE_UNKNOWN = 0
	uint8 PRIMARY_STATE_UNCONFIGURED = 1
	uint8 PRIMARY_STATE_INACTIVE = 2
	uint8 PRIMARY_STATE_ACTIVE = 3
	uint8 PRIMARY_STATE_FINALIZED = 4
	uint8 TRANSITION_STATE_CONFIGURING = 10
	uint8 TRANSITION_STATE_CLEANINGUP = 11
	uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
	uint8 TRANSITION_STATE_ACTIVATING = 13
	uint8 TRANSITION_STATE_DEACTIVATING = 14
	uint8 TRANSITION_STATE_ERRORPROCESSING = 15
	uint8 id
	string label
- **/joint_states**:
  - Type: sensor_msgs/msg/JointState
  - **Type Details:**
# This is a message that holds data to describe the state of a set of torque controlled joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

string[] name
float64[] position
float64[] velocity
float64[] effort
- **/joint_trajectory_controller/controller_state**:
  - Type: control_msgs/msg/JointTrajectoryControllerState
  - **Type Details:**
# This message presents current controller state of JTC

# Header timestamp should be update time of controller state
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

string[] joint_names
# The set point, that is, desired state.
trajectory_msgs/JointTrajectoryPoint reference
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# Current value of the process (ie: latest sensor measurement on the controlled value).
trajectory_msgs/JointTrajectoryPoint feedback
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# The error of the controlled value, essentially reference - feedback (for a regular PID implementation).
trajectory_msgs/JointTrajectoryPoint error
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# Current output of the controller.
trajectory_msgs/JointTrajectoryPoint output
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# -- deprecated --
trajectory_msgs/JointTrajectoryPoint desired
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
trajectory_msgs/JointTrajectoryPoint actual
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec

string[] multi_dof_joint_names
# The set point, that is, desired state.
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_reference
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# Current value of the process (ie: latest sensor measurement on the controlled value).
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_feedback
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# The error of the controlled value, essentially reference - feedback (for a regular PID implementation).
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_error
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# Current output of the controller.
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_output
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# -- deprecated --
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_desired
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_actual
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
- **/joint_trajectory_controller/joint_trajectory**:
  - Type: trajectory_msgs/msg/JointTrajectory
  - **Type Details:**
# The header is used to specify the coordinate frame and the reference time for
# the trajectory durations
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# The names of the active joints in each trajectory point. These names are
# ordered and must correspond to the values in each trajectory point.
string[] joint_names

# Array of trajectory points, which describe the positions, velocities,
# accelerations and/or efforts of the joints at each time point.
JointTrajectoryPoint[] points
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
- **/joint_trajectory_controller/state**:
  - Type: control_msgs/msg/JointTrajectoryControllerState
  - **Type Details:**
# This message presents current controller state of JTC

# Header timestamp should be update time of controller state
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

string[] joint_names
# The set point, that is, desired state.
trajectory_msgs/JointTrajectoryPoint reference
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# Current value of the process (ie: latest sensor measurement on the controlled value).
trajectory_msgs/JointTrajectoryPoint feedback
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# The error of the controlled value, essentially reference - feedback (for a regular PID implementation).
trajectory_msgs/JointTrajectoryPoint error
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# Current output of the controller.
trajectory_msgs/JointTrajectoryPoint output
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# -- deprecated --
trajectory_msgs/JointTrajectoryPoint desired
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
trajectory_msgs/JointTrajectoryPoint actual
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec

string[] multi_dof_joint_names
# The set point, that is, desired state.
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_reference
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# Current value of the process (ie: latest sensor measurement on the controlled value).
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_feedback
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# The error of the controlled value, essentially reference - feedback (for a regular PID implementation).
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_error
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# Current output of the controller.
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_output
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# -- deprecated --
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_desired
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_actual
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
- **/joint_trajectory_controller/transition_event**:
  - Type: lifecycle_msgs/msg/TransitionEvent
  - **Type Details:**
# The time point at which this event occurred.
uint64 timestamp

# The id and label of this transition event.
Transition transition
	uint8 TRANSITION_CREATE = 0
	uint8 TRANSITION_CONFIGURE = 1
	uint8 TRANSITION_CLEANUP = 2
	uint8 TRANSITION_ACTIVATE = 3
	uint8 TRANSITION_DEACTIVATE = 4
	uint8 TRANSITION_UNCONFIGURED_SHUTDOWN  = 5
	uint8 TRANSITION_INACTIVE_SHUTDOWN = 6
	uint8 TRANSITION_ACTIVE_SHUTDOWN = 7
	uint8 TRANSITION_DESTROY = 8
	uint8 TRANSITION_ON_CONFIGURE_SUCCESS = 10
	uint8 TRANSITION_ON_CONFIGURE_FAILURE = 11
	uint8 TRANSITION_ON_CONFIGURE_ERROR = 12
	uint8 TRANSITION_ON_CLEANUP_SUCCESS = 20
	uint8 TRANSITION_ON_CLEANUP_FAILURE = 21
	uint8 TRANSITION_ON_CLEANUP_ERROR = 22
	uint8 TRANSITION_ON_ACTIVATE_SUCCESS = 30
	uint8 TRANSITION_ON_ACTIVATE_FAILURE = 31
	uint8 TRANSITION_ON_ACTIVATE_ERROR = 32
	uint8 TRANSITION_ON_DEACTIVATE_SUCCESS = 40
	uint8 TRANSITION_ON_DEACTIVATE_FAILURE = 41
	uint8 TRANSITION_ON_DEACTIVATE_ERROR = 42
	uint8 TRANSITION_ON_SHUTDOWN_SUCCESS = 50
	uint8 TRANSITION_ON_SHUTDOWN_FAILURE = 51
	uint8 TRANSITION_ON_SHUTDOWN_ERROR = 52
	uint8 TRANSITION_ON_ERROR_SUCCESS = 60
	uint8 TRANSITION_ON_ERROR_FAILURE = 61
	uint8 TRANSITION_ON_ERROR_ERROR = 62
	uint8 TRANSITION_CALLBACK_SUCCESS = 97
	uint8 TRANSITION_CALLBACK_FAILURE = 98
	uint8 TRANSITION_CALLBACK_ERROR = 99
	##
	##
	uint8 id
	string label

# The starting state from which this event transitioned.
State start_state
	uint8 PRIMARY_STATE_UNKNOWN = 0
	uint8 PRIMARY_STATE_UNCONFIGURED = 1
	uint8 PRIMARY_STATE_INACTIVE = 2
	uint8 PRIMARY_STATE_ACTIVE = 3
	uint8 PRIMARY_STATE_FINALIZED = 4
	uint8 TRANSITION_STATE_CONFIGURING = 10
	uint8 TRANSITION_STATE_CLEANINGUP = 11
	uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
	uint8 TRANSITION_STATE_ACTIVATING = 13
	uint8 TRANSITION_STATE_DEACTIVATING = 14
	uint8 TRANSITION_STATE_ERRORPROCESSING = 15
	uint8 id
	string label

# The end state of this transition event.
State goal_state
	uint8 PRIMARY_STATE_UNKNOWN = 0
	uint8 PRIMARY_STATE_UNCONFIGURED = 1
	uint8 PRIMARY_STATE_INACTIVE = 2
	uint8 PRIMARY_STATE_ACTIVE = 3
	uint8 PRIMARY_STATE_FINALIZED = 4
	uint8 TRANSITION_STATE_CONFIGURING = 10
	uint8 TRANSITION_STATE_CLEANINGUP = 11
	uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
	uint8 TRANSITION_STATE_ACTIVATING = 13
	uint8 TRANSITION_STATE_DEACTIVATING = 14
	uint8 TRANSITION_STATE_ERRORPROCESSING = 15
	uint8 id
	string label
- **/odom**:
  - Type: nav_msgs/msg/Odometry
  - **Type Details:**
# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose
	Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	float64[36] covariance

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
	Twist twist
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	float64[36] covariance
- **/parameter_events**:
  - Type: rcl_interfaces/msg/ParameterEvent
  - **Type Details:**
# This message contains a parameter event.
# Because the parameter event was an atomic update, a specific parameter name
# can only be in one of the three sets.

# The time stamp when this parameter event occurred.
builtin_interfaces/Time stamp
	int32 sec
	uint32 nanosec

# Fully qualified ROS path to node.
string node

# New parameters that have been set for this node.
Parameter[] new_parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

# Parameters that have been changed during this event.
Parameter[] changed_parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

# Parameters that have been deleted during this event.
Parameter[] deleted_parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value
- **/performance_metrics**:
  - Type: gazebo_msgs/msg/PerformanceMetrics
  - **Type Details:**
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

float64 real_time_factor
gazebo_msgs/SensorPerformanceMetric[] sensors
	string name
	float64 sim_update_rate
	float64 real_update_rate
	float64 fps
- **/robot_description**:
  - Type: std_msgs/msg/String
  - **Type Details:**
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string data
- **/rosout**:
  - Type: rcl_interfaces/msg/Log
  - **Type Details:**
##
## Severity level constants
##
## These logging levels follow the Python Standard
## https://docs.python.org/3/library/logging.html#logging-levels
## And are implemented in rcutils as well
## https://github.com/ros2/rcutils/blob/35f29850064e0c33a4063cbc947ebbfeada11dba/include/rcutils/logging.h#L164-L172
## This leaves space for other standard logging levels to be inserted in the middle in the future,
## as well as custom user defined levels.
## Since there are several other logging enumeration standard for different implementations,
## other logging implementations may need to provide level mappings to match their internal implementations.
##

# Debug is for pedantic information, which is useful when debugging issues.
byte DEBUG=10

# Info is the standard informational level and is used to report expected
# information.
byte INFO=20

# Warning is for information that may potentially cause issues or possibly unexpected
# behavior.
byte WARN=30

# Error is for information that this node cannot resolve.
byte ERROR=40

# Information about a impending node shutdown.
byte FATAL=50

##
## Fields
##

# Timestamp when this message was generated by the node.
builtin_interfaces/Time stamp
	int32 sec
	uint32 nanosec

# Corresponding log level, see above definitions.
uint8 level

# The name representing the logger this message came from.
string name

# The full log message.
string msg

# The file the message came from.
string file

# The function the message came from.
string function

# The line in the file the message came from.
uint32 line
- **/scan**:
  - Type: sensor_msgs/msg/LaserScan
  - **Type Details:**
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
- **/scan2**:
  - Type: sensor_msgs/msg/LaserScan
  - **Type Details:**
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
- **/tf**:
  - Type: tf2_msgs/msg/TFMessage
  - **Type Details:**
geometry_msgs/TransformStamped[] transforms
	#
	#
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string child_frame_id
	Transform transform
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
- **/tf_static**:
  - Type: tf2_msgs/msg/TFMessage
  - **Type Details:**
geometry_msgs/TransformStamped[] transforms
	#
	#
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string child_frame_id
	Transform transform
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1

## Services
- **/apply_joint_effort**:
  - Type: gazebo_msgs/srv/ApplyJointEffort
  - **Type Details:**
# set urdf joint effort
string joint_name                       # joint to apply wrench (linear force and torque)
float64 effort                          # effort to apply
builtin_interfaces/Time start_time      # optional wrench application start time (seconds)
	int32 sec
	uint32 nanosec
                                        # if start_time < current time, start as soon as possible
builtin_interfaces/Duration  duration   # optional duration of wrench application time (seconds)
	int32 sec
	uint32 nanosec
                                        # if duration < 0, apply wrench continuously without end
                                        # if duration = 0, do nothing
                                        # if duration < step size, assume step size and
                                        # display warning in status_message
---
bool success                            # return true if effort application is successful
string status_message                   # comments if available
- **/apply_link_wrench**:
  - Type: gazebo_msgs/srv/ApplyLinkWrench
  - **Type Details:**
# Apply Wrench to Gazebo Link.
# via the callback mechanism
# all Gazebo operations are made in world frame
string link_name                          # Gazebo link to apply wrench (linear force and torque)
                                          # wrench is applied in the gazebo world by default
                                          # link names are prefixed by model name, e.g. pr2::base_link
string reference_frame                    # wrench is defined in the reference frame of this entity
                                          # use inertial frame if left empty
                                          # frame names are links prefixed by model name, e.g. pr2::base_link
geometry_msgs/Point  reference_point      # wrench is defined at this location in the reference frame
	float64 x
	float64 y
	float64 z
geometry_msgs/Wrench wrench               # wrench applied to the origin of the link
	Vector3  force
		float64 x
		float64 y
		float64 z
	Vector3  torque
		float64 x
		float64 y
		float64 z
builtin_interfaces/Time start_time        # (optional) wrench application start time (seconds)
	int32 sec
	uint32 nanosec
                                          # if start_time is not specified, or
                                          # start_time < current time, start as soon as possible
builtin_interfaces/Duration duration      # optional duration of wrench application time (seconds)
	int32 sec
	uint32 nanosec
                                          # if duration < 0, apply wrench continuously without end
                                          # if duration = 0, do nothing
                                          # if duration < step size, apply wrench
                                          # for one step size
---
bool success                              # return true if set wrench successful
string status_message                     # comments if available
- **/clear_joint_efforts**:
  - Type: gazebo_msgs/srv/JointRequest
  - **Type Details:**
string joint_name   # name of the joint requested
---
- **/clear_link_wrenches**:
  - Type: gazebo_msgs/srv/LinkRequest
  - **Type Details:**
string link_name   # name of the link requested. link names are prefixed by model name, e.g. pr2::base_link
---
- **/controller_manager/configure_controller**:
  - Type: controller_manager_msgs/srv/ConfigureController
  - **Type Details:**
# The ConfigureController service allows you to configure a single controller
# inside controller_manager

# To configure a controller, specify the "name" of the controller.
# The return value "ok" indicates if the controller was successfully
# configured or not.

string name
---
bool ok
- **/controller_manager/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/controller_manager/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/controller_manager/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/controller_manager/list_controller_types**:
  - Type: controller_manager_msgs/srv/ListControllerTypes
  - **Type Details:**
# The ListControllers service returns a list of controller types that are known
# to the controller manager plugin mechanism.

---
string[] types
string[] base_classes
- **/controller_manager/list_controllers**:
  - Type: controller_manager_msgs/srv/ListControllers
  - **Type Details:**
# The ListControllers service returns a list of controller names/states/types of the
# controllers that are loaded inside the controller_manager.

---
controller_manager_msgs/ControllerState[] controller
	string name        #
	string state        #
	string type        #
	string[] claimed_interfaces        #
	string[] required_command_interfaces        #
	string[] required_state_interfaces        #
	bool is_chainable        #
	bool is_chained        #
	string[] reference_interfaces        #
	ChainConnection[] chain_connections        #
		string name        #
		string[] reference_interfaces        #
- **/controller_manager/list_hardware_components**:
  - Type: controller_manager_msgs/srv/ListHardwareComponents
  - **Type Details:**
# The ListHardwareComponents service returns a list of hardware HardwareComponentsState.
# This will convey name, component_type, state and type of the components
# that are loaded inside the resource_manager.

---
HardwareComponentState[] component
	string name
	string type
	string class_type
	lifecycle_msgs/State state
		uint8 PRIMARY_STATE_UNKNOWN = 0
		uint8 PRIMARY_STATE_UNCONFIGURED = 1
		uint8 PRIMARY_STATE_INACTIVE = 2
		uint8 PRIMARY_STATE_ACTIVE = 3
		uint8 PRIMARY_STATE_FINALIZED = 4
		uint8 TRANSITION_STATE_CONFIGURING = 10
		uint8 TRANSITION_STATE_CLEANINGUP = 11
		uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
		uint8 TRANSITION_STATE_ACTIVATING = 13
		uint8 TRANSITION_STATE_DEACTIVATING = 14
		uint8 TRANSITION_STATE_ERRORPROCESSING = 15
		uint8 id
		string label
	HardwareInterface[] command_interfaces
		string name
		bool is_available
		bool is_claimed
	HardwareInterface[] state_interfaces
		string name
		bool is_available
		bool is_claimed
- **/controller_manager/list_hardware_interfaces**:
  - Type: controller_manager_msgs/srv/ListHardwareInterfaces
  - **Type Details:**
---
HardwareInterface[] command_interfaces
	string name
	bool is_available
	bool is_claimed
HardwareInterface[] state_interfaces
	string name
	bool is_available
	bool is_claimed
- **/controller_manager/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/controller_manager/load_controller**:
  - Type: controller_manager_msgs/srv/LoadController
  - **Type Details:**
# The LoadController service allows you to load a single controller
# inside controller_manager

# To load a controller, specify the "name" of the controller.
# The return value "ok" indicates if the controller was successfully
# constructed and initialized or not.

string name
---
bool ok
- **/controller_manager/reload_controller_libraries**:
  - Type: controller_manager_msgs/srv/ReloadControllerLibraries
  - **Type Details:**
# The ReloadControllerLibraries service will reload all controllers that are available in
# the system as plugins

# Reloading libraries only works if there are no controllers loaded. If there
# are still some controllers loaded, the reloading will fail.
# If this bool is set to true, all loaded controllers will get
# killed automatically, and the reloading can succeed.
bool force_kill
---
bool ok
- **/controller_manager/set_hardware_component_state**:
  - Type: controller_manager_msgs/srv/SetHardwareComponentState
  - **Type Details:**
# The SetHardwareComponentState service allows to control life-cycle of a single hardware component.
# Supported states are defined in the design document of LifecycleNodes available at:
# https://design.ros2.org/articles/node_lifecycle.html
# To control life-cycle of a hardware component, specify its "name" and "target_state".
# Target state may be defined by "id" using a constant from `lifecycle_msgs/msg/State` or a label
# using definitions from `hardware_interface/types/lifecycle_state_names.hpp` file.
# The return value "ok" indicates if the component has successfully changed its state to "target_state".
# The return value "state" returns current state of the hardware component.

string name
lifecycle_msgs/State target_state
	uint8 PRIMARY_STATE_UNKNOWN = 0
	uint8 PRIMARY_STATE_UNCONFIGURED = 1
	uint8 PRIMARY_STATE_INACTIVE = 2
	uint8 PRIMARY_STATE_ACTIVE = 3
	uint8 PRIMARY_STATE_FINALIZED = 4
	uint8 TRANSITION_STATE_CONFIGURING = 10
	uint8 TRANSITION_STATE_CLEANINGUP = 11
	uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
	uint8 TRANSITION_STATE_ACTIVATING = 13
	uint8 TRANSITION_STATE_DEACTIVATING = 14
	uint8 TRANSITION_STATE_ERRORPROCESSING = 15
	uint8 id
	string label
---
bool ok
lifecycle_msgs/State state
	uint8 PRIMARY_STATE_UNKNOWN = 0
	uint8 PRIMARY_STATE_UNCONFIGURED = 1
	uint8 PRIMARY_STATE_INACTIVE = 2
	uint8 PRIMARY_STATE_ACTIVE = 3
	uint8 PRIMARY_STATE_FINALIZED = 4
	uint8 TRANSITION_STATE_CONFIGURING = 10
	uint8 TRANSITION_STATE_CLEANINGUP = 11
	uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
	uint8 TRANSITION_STATE_ACTIVATING = 13
	uint8 TRANSITION_STATE_DEACTIVATING = 14
	uint8 TRANSITION_STATE_ERRORPROCESSING = 15
	uint8 id
	string label
- **/controller_manager/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/controller_manager/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/controller_manager/switch_controller**:
  - Type: controller_manager_msgs/srv/SwitchController
  - **Type Details:**
# The SwitchController service allows you deactivate a number of controllers
# and activate a number of controllers, all in one single timestep of the
# controller manager's control loop.

# To switch controllers, specify
#  * the list of controller names to activate,
#  * the list of controller names to deactivate, and
#  * the strictness (BEST_EFFORT or STRICT)
#    * STRICT means that switching will fail if anything goes wrong (an invalid
#      controller name, a controller that failed to activate, etc. )
#    * BEST_EFFORT means that even when something goes wrong with on controller,
#      the service will still try to activate/stop the remaining controllers
#  * activate the controllers as soon as their hardware dependencies are ready, will
#    wait for all interfaces to be ready otherwise
#  * the timeout before aborting pending controllers. Zero for infinite

# The return value "ok" indicates if the controllers were switched
# successfully or not.  The meaning of success depends on the
# specified strictness.


string[] activate_controllers
string[] deactivate_controllers
string[] start_controllers       # DEPRECATED: Use activate_controllers filed instead
string[] stop_controllers        # DEPRECATED: Use deactivate_controllers filed instead
int32 strictness
int32 BEST_EFFORT=1
int32 STRICT=2
bool start_asap                 # DEPRECATED: Use activate_asap filed instead
bool activate_asap
builtin_interfaces/Duration timeout
	int32 sec
	uint32 nanosec
---
bool ok
- **/controller_manager/unload_controller**:
  - Type: controller_manager_msgs/srv/UnloadController
  - **Type Details:**
# The UnloadController service allows you to unload a single controller
# from controller_manager

# To unload a controller, specify the "name" of the controller.
# The return value "ok" indicates if the controller was successfully
# unloaded or not

string name
---
bool ok
- **/delete_entity**:
  - Type: gazebo_msgs/srv/DeleteEntity
  - **Type Details:**
string name                       # Name of the Gazebo entity to be deleted. This can be either
                                  # a model or a light.
---
bool success                      # Return true if deletion is successful.
string status_message             # Comments if available.
- **/gazebo/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/gazebo/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/gazebo/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/gazebo/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/gazebo/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/gazebo/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/gazebo_ros2_control/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/gazebo_ros2_control/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/gazebo_ros2_control/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/gazebo_ros2_control/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/gazebo_ros2_control/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/gazebo_ros2_control/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/get_model_list**:
  - Type: gazebo_msgs/srv/GetModelList
  - **Type Details:**
---
std_msgs/Header header               # Standard metadata for higher-level stamped data types.
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
                                     # * header.stamp Simulation time when data was collected.
string[] model_names                 # list of models in the world
bool success                         # return true if get successful
- **/joint_state_broadcaster/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/joint_state_broadcaster/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/joint_state_broadcaster/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/joint_state_broadcaster/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/joint_state_broadcaster/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/joint_state_broadcaster/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/joint_trajectory_controller/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/joint_trajectory_controller/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/joint_trajectory_controller/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/joint_trajectory_controller/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/joint_trajectory_controller/query_state**:
  - Type: control_msgs/srv/QueryTrajectoryState
  - **Type Details:**
builtin_interfaces/Time time
	int32 sec
	uint32 nanosec
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

string[] name
float64[] position
float64[] velocity
float64[] acceleration
- **/joint_trajectory_controller/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/joint_trajectory_controller/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/lidar_1_sensor_plugin/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/lidar_1_sensor_plugin/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/lidar_1_sensor_plugin/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/lidar_1_sensor_plugin/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/lidar_1_sensor_plugin/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/lidar_1_sensor_plugin/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/lidar_2_sensor_plugin/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/lidar_2_sensor_plugin/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/lidar_2_sensor_plugin/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/lidar_2_sensor_plugin/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/lidar_2_sensor_plugin/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/lidar_2_sensor_plugin/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/object_controller/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/object_controller/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/object_controller/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/object_controller/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/object_controller/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/object_controller/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/pause_physics**:
  - Type: std_srvs/srv/Empty
  - **Type Details:**
---
- **/reset_simulation**:
  - Type: std_srvs/srv/Empty
  - **Type Details:**
---
- **/reset_world**:
  - Type: std_srvs/srv/Empty
  - **Type Details:**
---
- **/robot_state_publisher/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/robot_state_publisher/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/robot_state_publisher/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/robot_state_publisher/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/robot_state_publisher/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/robot_state_publisher/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/ros2_inspector/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/ros2_inspector/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/ros2_inspector/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/ros2_inspector/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/ros2_inspector/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/ros2_inspector/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/spawn_entity**:
  - Type: gazebo_msgs/srv/SpawnEntity
  - **Type Details:**
string name                       # Name of the entity to be spawned (optional).
string xml                        # Entity XML description as a string, either URDF or SDF.
string robot_namespace            # Spawn robot and all ROS interfaces under this namespace
geometry_msgs/Pose initial_pose   # Initial entity pose.
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1
string reference_frame            # initial_pose is defined relative to the frame of this entity.
                                  # If left empty or "world" or "map", then gazebo world frame is
                                  # used.
                                  # If non-existent entity is specified, an error is returned
                                  # and the entity is not spawned.
---
bool success                      # Return true if spawned successfully.
string status_message             # Comments if available.
- **/teleop/describe_parameters**:
  - Type: rcl_interfaces/srv/DescribeParameters
  - **Type Details:**
# A list of parameters of which to get the descriptor.
string[] names

---
# A list of the descriptors of all parameters requested in the same order
# as they were requested. This list has the same length as the list of
# parameters requested.
ParameterDescriptor[] descriptors
	string name
	uint8 type
	string description
	#
	string additional_constraints
	bool read_only false
	bool dynamic_typing false
	#
	FloatingPointRange[<=1] floating_point_range
		float64 from_value
		float64 to_value
		#
		#
		#
		#
		float64 step
	IntegerRange[<=1] integer_range
		int64 from_value
		int64 to_value
		#
		#
		#
		uint64 step
- **/teleop/get_parameter_types**:
  - Type: rcl_interfaces/srv/GetParameterTypes
  - **Type Details:**
# A list of parameter names.
# TODO(wjwwood): link to parameter naming rules.
string[] names

---
# List of types which is the same length and order as the provided names.
#
# The type enum is defined in ParameterType.msg. ParameterType.PARAMETER_NOT_SET
# indicates that the parameter is not currently set.
uint8[] types
- **/teleop/get_parameters**:
  - Type: rcl_interfaces/srv/GetParameters
  - **Type Details:**
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
	uint8 type
	bool bool_value
	int64 integer_value
	float64 double_value
	string string_value
	byte[] byte_array_value
	bool[] bool_array_value
	int64[] integer_array_value
	float64[] double_array_value
	string[] string_array_value
- **/teleop/list_parameters**:
  - Type: rcl_interfaces/srv/ListParameters
  - **Type Details:**
# Recursively get parameters with unlimited depth.
uint64 DEPTH_RECURSIVE=0

# The list of parameter prefixes to query.
string[] prefixes

# Relative depth from given prefixes to return.
#
# Use DEPTH_RECURSIVE to get the recursive parameters and prefixes for each prefix.
uint64 depth

---
# The list of parameter names and their prefixes.
ListParametersResult result
	string[] names
	string[] prefixes
- **/teleop/set_parameters**:
  - Type: rcl_interfaces/srv/SetParameters
  - **Type Details:**
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
- **/teleop/set_parameters_atomically**:
  - Type: rcl_interfaces/srv/SetParametersAtomically
  - **Type Details:**
# A list of parameters to set atomically.
#
# This call will either set all values, or none of the values.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting all of the parameters succeeded or not and why.
SetParametersResult result
	bool successful
	string reason
- **/unpause_physics**:
  - Type: std_srvs/srv/Empty
  - **Type Details:**
---

## Actions
- **/joint_trajectory_controller/follow_joint_trajectory**: (action_types)
  - **Message Type Details:**
# The trajectory for all revolute, continuous or prismatic joints
trajectory_msgs/JointTrajectory trajectory
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string[] joint_names
	JointTrajectoryPoint[] points
		float64[] positions
		float64[] velocities
		float64[] accelerations
		float64[] effort
		builtin_interfaces/Duration time_from_start
			int32 sec
			uint32 nanosec
# The trajectory for all planar or floating joints (i.e. individual joints with more than one DOF)
trajectory_msgs/MultiDOFJointTrajectory multi_dof_trajectory
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string[] joint_names
	MultiDOFJointTrajectoryPoint[] points
		geometry_msgs/Transform[] transforms
			Vector3 translation
				float64 x
				float64 y
				float64 z
			Quaternion rotation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		geometry_msgs/Twist[] velocities
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Twist[] accelerations
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		builtin_interfaces/Duration time_from_start
			int32 sec
			uint32 nanosec

# Tolerances for the trajectory.  If the measured joint values fall
# outside the tolerances the trajectory goal is aborted.  Any
# tolerances that are not specified (by being omitted or set to 0) are
# set to the defaults for the action server (often taken from the
# parameter server).

# Tolerances applied to the joints as the trajectory is executed.  If
# violated, the goal aborts with error_code set to
# PATH_TOLERANCE_VIOLATED.
JointTolerance[] path_tolerance
	#
	string name
	float64 position  #
	float64 velocity  #
	float64 acceleration  #
JointComponentTolerance[] component_path_tolerance
	uint16 X_AXIS=1
	uint16 Y_AXIS=2
	uint16 Z_AXIS=3
	uint16 TRANSLATION=4
	uint16 ROTATION=5
	string joint_name
	uint16 component
	float64 position
	float64 velocity
	float64 acceleration

# To report success, the joints must be within goal_tolerance of the
# final trajectory value.  The goal must be achieved by time the
# trajectory ends plus goal_time_tolerance.  (goal_time_tolerance
# allows some leeway in time, so that the trajectory goal can still
# succeed even if the joints reach the goal some time after the
# precise end time of the trajectory).
#
# If the joints are not within goal_tolerance after "trajectory finish
# time" + goal_time_tolerance, the goal aborts with error_code set to
# GOAL_TOLERANCE_VIOLATED
JointTolerance[] goal_tolerance
	#
	string name
	float64 position  #
	float64 velocity  #
	float64 acceleration  #
JointComponentTolerance[] component_goal_tolerance
	uint16 X_AXIS=1
	uint16 Y_AXIS=2
	uint16 Z_AXIS=3
	uint16 TRANSLATION=4
	uint16 ROTATION=5
	string joint_name
	uint16 component
	float64 position
	float64 velocity
	float64 acceleration
builtin_interfaces/Duration goal_time_tolerance
	int32 sec
	uint32 nanosec

---
int32 error_code
int32 SUCCESSFUL = 0
int32 INVALID_GOAL = -1
int32 INVALID_JOINTS = -2
int32 OLD_HEADER_TIMESTAMP = -3
int32 PATH_TOLERANCE_VIOLATED = -4
int32 GOAL_TOLERANCE_VIOLATED = -5

# Human readable description of the error code. Contains complementary
# information that is especially useful when execution fails, for instance:
# - INVALID_GOAL: The reason for the invalid goal (e.g., the requested
#   trajectory is in the past).
# - INVALID_JOINTS: The mismatch between the expected controller joints
#   and those provided in the goal.
# - PATH_TOLERANCE_VIOLATED and GOAL_TOLERANCE_VIOLATED: Which joint
#   violated which tolerance, and by how much.
string error_string

---
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
string[] joint_names
trajectory_msgs/JointTrajectoryPoint desired
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
trajectory_msgs/JointTrajectoryPoint actual
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
trajectory_msgs/JointTrajectoryPoint error
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec

string[] multi_dof_joint_names
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_desired
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_actual
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_error
	geometry_msgs/Transform[] transforms
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist[] velocities
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	geometry_msgs/Twist[] accelerations
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
