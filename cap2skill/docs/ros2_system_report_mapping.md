# ROS2 System Report created on: 2025-02-21 17:28:35
## Topics
- **/attached_collision_object**:
  - Type: moveit_msgs/msg/AttachedCollisionObject
  - **Type Details:**
# The CollisionObject will be attached with a fixed joint to this link
string link_name

#This contains the actual shapes and poses for the CollisionObject
#to be attached to the link
#If action is remove and no object.id is set, all objects
#attached to the link indicated by link_name will be removed
CollisionObject object
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	geometry_msgs/Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	string id
	object_recognition_msgs/ObjectType type
		string key
		string db
	shape_msgs/SolidPrimitive[] primitives
		uint8 BOX=1
		uint8 SPHERE=2
		uint8 CYLINDER=3
		uint8 CONE=4
		uint8 PRISM=5
		uint8 type
		float64[<=3] dimensions  #
		uint8 BOX_X=0
		uint8 BOX_Y=1
		uint8 BOX_Z=2
		uint8 SPHERE_RADIUS=0
		uint8 CYLINDER_HEIGHT=0
		uint8 CYLINDER_RADIUS=1
		uint8 CONE_HEIGHT=0
		uint8 CONE_RADIUS=1
		uint8 PRISM_HEIGHT=0
		geometry_msgs/Polygon polygon
			Point32[] points
				#
				#
				float32 x
				float32 y
				float32 z
	geometry_msgs/Pose[] primitive_poses
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	shape_msgs/Mesh[] meshes
		MeshTriangle[] triangles
			uint32[3] vertex_indices
		geometry_msgs/Point[] vertices
			float64 x
			float64 y
			float64 z
	geometry_msgs/Pose[] mesh_poses
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	shape_msgs/Plane[] planes
		#
		float64[4] coef
	geometry_msgs/Pose[] plane_poses
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	string[] subframe_names
	geometry_msgs/Pose[] subframe_poses
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	byte ADD=0
	byte REMOVE=1
	byte APPEND=2
	byte MOVE=3
	byte operation

# The set of links that the attached objects are allowed to touch
# by default - the link_name is already considered by default
string[] touch_links

# If certain links were placed in a particular posture for this object to remain attached
# (e.g., an end effector closing around an object), the posture necessary for releasing
# the object is stored here
trajectory_msgs/JointTrajectory detach_posture
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

# The weight of the attached object, if known
float64 weight
- **/clicked_point**:
  - Type: geometry_msgs/msg/PointStamped
  - **Type Details:**
# This represents a Point with reference coordinate frame and timestamp

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
Point point
	float64 x
	float64 y
	float64 z
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
- **/collision_object**:
  - Type: moveit_msgs/msg/CollisionObject
  - **Type Details:**
# a header, used for interpreting the poses
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# DISCLAIMER: This field is not in use yet and all other poses
# are still interpreted in the header frame.
# https://github.com/ros-planning/moveit/pull/2037
# implements the actual logic for this field.
# ---
# The object's pose relative to the header frame.
# The shapes and subframe poses are defined relative to this pose.
geometry_msgs/Pose pose
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1

# The id of the object (name used in MoveIt)
string id

# The object type in a database of known objects
object_recognition_msgs/ObjectType type
	string key
	string db

# The collision geometries associated with the object.
# Their poses are with respect to the object's pose

# Solid geometric primitives
shape_msgs/SolidPrimitive[] primitives
	uint8 BOX=1
	uint8 SPHERE=2
	uint8 CYLINDER=3
	uint8 CONE=4
	uint8 PRISM=5
	uint8 type
	float64[<=3] dimensions  #
	uint8 BOX_X=0
	uint8 BOX_Y=1
	uint8 BOX_Z=2
	uint8 SPHERE_RADIUS=0
	uint8 CYLINDER_HEIGHT=0
	uint8 CYLINDER_RADIUS=1
	uint8 CONE_HEIGHT=0
	uint8 CONE_RADIUS=1
	uint8 PRISM_HEIGHT=0
	geometry_msgs/Polygon polygon
		Point32[] points
			#
			#
			float32 x
			float32 y
			float32 z
geometry_msgs/Pose[] primitive_poses
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1

# Meshes
shape_msgs/Mesh[] meshes
	MeshTriangle[] triangles
		uint32[3] vertex_indices
	geometry_msgs/Point[] vertices
		float64 x
		float64 y
		float64 z
geometry_msgs/Pose[] mesh_poses
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1

# Bounding planes (equation is specified, but the plane can be oriented using an additional pose)
shape_msgs/Plane[] planes
	#
	float64[4] coef
geometry_msgs/Pose[] plane_poses
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1

# Named subframes on the object. Use these to define points of interest on the object that you want
# to plan with (e.g. "tip", "spout", "handle"). The id of the object will be prepended to the subframe.
# If an object with the id "screwdriver" and a subframe "tip" is in the scene, you can use the frame
# "screwdriver/tip" for planning.
# The length of the subframe_names and subframe_poses has to be identical.
string[] subframe_names
geometry_msgs/Pose[] subframe_poses
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1

# Adds the object to the planning scene. If the object previously existed, it is replaced.
byte ADD=0

# Removes the object from the environment entirely (everything that matches the specified id)
byte REMOVE=1

# Append to an object that already exists in the planning scene. If the object does not exist, it is added.
byte APPEND=2

# If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)
# if solely moving the object is desired
byte MOVE=3

# Operation to be performed
byte operation
- **/display_contacts**:
  - Type: visualization_msgs/msg/MarkerArray
  - **Type Details:**
Marker[] markers
	#
	int32 ARROW=0
	int32 CUBE=1
	int32 SPHERE=2
	int32 CYLINDER=3
	int32 LINE_STRIP=4
	int32 LINE_LIST=5
	int32 CUBE_LIST=6
	int32 SPHERE_LIST=7
	int32 POINTS=8
	int32 TEXT_VIEW_FACING=9
	int32 MESH_RESOURCE=10
	int32 TRIANGLE_LIST=11
	int32 ADD=0
	int32 MODIFY=0
	int32 DELETE=2
	int32 DELETEALL=3
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string ns
	int32 id
	int32 type
	int32 action
	geometry_msgs/Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Vector3 scale
		float64 x
		float64 y
		float64 z
	std_msgs/ColorRGBA color
		float32 r
		float32 g
		float32 b
		float32 a
	builtin_interfaces/Duration lifetime
		int32 sec
		uint32 nanosec
	bool frame_locked
	geometry_msgs/Point[] points
		float64 x
		float64 y
		float64 z
	std_msgs/ColorRGBA[] colors
		float32 r
		float32 g
		float32 b
		float32 a
	string texture_resource
	sensor_msgs/CompressedImage texture
		std_msgs/Header header #
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		                             # Header frame_id should be optical frame of camera
		                             # origin of frame should be optical center of cameara
		                             # +x should point to the right in the image
		                             # +y should point down in the image
		                             # +z should point into to plane of the image
		string format                #
		                             #   Acceptable values:
		                             #     jpeg, png, tiff
		uint8[] data                 #
	UVCoordinate[] uv_coordinates
		float32 u
		float32 v
	string text
	string mesh_resource
	MeshFile mesh_file
		string filename
		uint8[] data
	bool mesh_use_embedded_materials
- **/display_planned_path**:
  - Type: moveit_msgs/msg/DisplayTrajectory
  - **Type Details:**
# The model id for which this path has been generated
string model_id

# The representation of the path contains position values for all the joints that are moving along the path; a sequence of trajectories may be specified
RobotTrajectory[] trajectory
	trajectory_msgs/JointTrajectory joint_trajectory
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
	trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
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

# The robot state is used to obtain positions for all/some of the joints of the robot.
# It is used by the path display node to determine the positions of the joints that are not specified in the joint path message above.
# If the robot state message contains joint position information for joints that are also mentioned in the joint path message, the positions in the joint path message will overwrite the positions specified in the robot state message.
RobotState trajectory_start
	sensor_msgs/JointState joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] name
		float64[] position
		float64[] velocity
		float64[] effort
	sensor_msgs/MultiDOFJointState multi_dof_joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] joint_names
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
		geometry_msgs/Twist[] twist
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Wrench[] wrench
			Vector3  force
				float64 x
				float64 y
				float64 z
			Vector3  torque
				float64 x
				float64 y
				float64 z
	AttachedCollisionObject[] attached_collision_objects
		string link_name
		CollisionObject object
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		string[] touch_links
		trajectory_msgs/JointTrajectory detach_posture
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
		float64 weight
	bool is_diff
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
- **/goal_pose**:
  - Type: geometry_msgs/msg/PoseStamped
  - **Type Details:**
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
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
- **/initialpose**:
  - Type: geometry_msgs/msg/PoseWithCovarianceStamped
  - **Type Details:**
# This expresses an estimated pose with a reference coordinate frame and timestamp

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
PoseWithCovariance pose
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
- **/monitored_planning_scene**:
  - Type: moveit_msgs/msg/PlanningScene
  - **Type Details:**
# name of planning scene
string name

# full robot state
RobotState robot_state
	sensor_msgs/JointState joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] name
		float64[] position
		float64[] velocity
		float64[] effort
	sensor_msgs/MultiDOFJointState multi_dof_joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] joint_names
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
		geometry_msgs/Twist[] twist
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Wrench[] wrench
			Vector3  force
				float64 x
				float64 y
				float64 z
			Vector3  torque
				float64 x
				float64 y
				float64 z
	AttachedCollisionObject[] attached_collision_objects
		string link_name
		CollisionObject object
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		string[] touch_links
		trajectory_msgs/JointTrajectory detach_posture
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
		float64 weight
	bool is_diff

# The name of the robot model this scene is for
string robot_model_name

#additional frames for duplicating tf (with respect to the planning frame)
geometry_msgs/TransformStamped[] fixed_frame_transforms
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

#full allowed collision matrix
AllowedCollisionMatrix allowed_collision_matrix
	string[] entry_names
	AllowedCollisionEntry[] entry_values
		bool[] enabled
	string[] default_entry_names
	bool[] default_entry_values

# all link paddings
LinkPadding[] link_padding
	string link_name
	float64 padding

# all link scales
LinkScale[] link_scale
	string link_name
	float64 scale

# Attached objects, collision objects, even the octomap or collision map can have
# colors associated to them. This array specifies them.
ObjectColor[] object_colors
	string id
	std_msgs/ColorRGBA color
		float32 r
		float32 g
		float32 b
		float32 a

# the collision map
PlanningSceneWorld world
	CollisionObject[] collision_objects
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		geometry_msgs/Pose pose
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		string id
		object_recognition_msgs/ObjectType type
			string key
			string db
		shape_msgs/SolidPrimitive[] primitives
			uint8 BOX=1
			uint8 SPHERE=2
			uint8 CYLINDER=3
			uint8 CONE=4
			uint8 PRISM=5
			uint8 type
			float64[<=3] dimensions  #
			uint8 BOX_X=0
			uint8 BOX_Y=1
			uint8 BOX_Z=2
			uint8 SPHERE_RADIUS=0
			uint8 CYLINDER_HEIGHT=0
			uint8 CYLINDER_RADIUS=1
			uint8 CONE_HEIGHT=0
			uint8 CONE_RADIUS=1
			uint8 PRISM_HEIGHT=0
			geometry_msgs/Polygon polygon
				Point32[] points
					#
					#
					float32 x
					float32 y
					float32 z
		geometry_msgs/Pose[] primitive_poses
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		shape_msgs/Mesh[] meshes
			MeshTriangle[] triangles
				uint32[3] vertex_indices
			geometry_msgs/Point[] vertices
				float64 x
				float64 y
				float64 z
		geometry_msgs/Pose[] mesh_poses
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		shape_msgs/Plane[] planes
			#
			float64[4] coef
		geometry_msgs/Pose[] plane_poses
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		string[] subframe_names
		geometry_msgs/Pose[] subframe_poses
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		byte ADD=0
		byte REMOVE=1
		byte APPEND=2
		byte MOVE=3
		byte operation
	octomap_msgs/OctomapWithPose octomap
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		geometry_msgs/Pose origin
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		octomap_msgs/Octomap octomap
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			bool binary
			string id
			float64 resolution
			int8[] data

# Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene
bool is_diff
- **/motion_plan_request**:
  - Type: moveit_msgs/msg/MotionPlanRequest
  - **Type Details:**
# This service contains the definition for a request to the motion
# planner and the output it provides

# Parameters for the workspace that the planner should work inside
WorkspaceParameters workspace_parameters
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	geometry_msgs/Vector3 min_corner
		float64 x
		float64 y
		float64 z
	geometry_msgs/Vector3 max_corner
		float64 x
		float64 y
		float64 z

# Starting state updates. If certain joints should be considered
# at positions other than the current ones, these positions should
# be set here
RobotState start_state
	sensor_msgs/JointState joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] name
		float64[] position
		float64[] velocity
		float64[] effort
	sensor_msgs/MultiDOFJointState multi_dof_joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] joint_names
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
		geometry_msgs/Twist[] twist
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Wrench[] wrench
			Vector3  force
				float64 x
				float64 y
				float64 z
			Vector3  torque
				float64 x
				float64 y
				float64 z
	AttachedCollisionObject[] attached_collision_objects
		string link_name
		CollisionObject object
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		string[] touch_links
		trajectory_msgs/JointTrajectory detach_posture
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
		float64 weight
	bool is_diff

# The possible goal states for the model to plan for. Each element of
# the array defines a goal region. The goal is achieved
# if the constraints for a particular region are satisfied
Constraints[] goal_constraints
	string name
	JointConstraint[] joint_constraints
		string joint_name
		float64 position
		float64 tolerance_above
		float64 tolerance_below
		float64 weight
	PositionConstraint[] position_constraints
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string link_name
		geometry_msgs/Vector3 target_point_offset
			float64 x
			float64 y
			float64 z
		BoundingVolume constraint_region
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
		float64 weight
	OrientationConstraint[] orientation_constraints
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		geometry_msgs/Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
		string link_name
		float64 absolute_x_axis_tolerance
		float64 absolute_y_axis_tolerance
		float64 absolute_z_axis_tolerance
		uint8 parameterization
		uint8 XYZ_EULER_ANGLES=0
		uint8 ROTATION_VECTOR=1
		float64 weight
	VisibilityConstraint[] visibility_constraints
		float64 target_radius
		geometry_msgs/PoseStamped target_pose
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
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
		int32 cone_sides
		geometry_msgs/PoseStamped sensor_pose
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
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
		float64 max_view_angle
		float64 max_range_angle
		uint8 SENSOR_Z=0
		uint8 SENSOR_Y=1
		uint8 SENSOR_X=2
		uint8 sensor_view_direction
		float64 weight

# No state at any point along the path in the produced motion plan will violate these constraints (this applies to all points, not just waypoints)
Constraints path_constraints
	string name
	JointConstraint[] joint_constraints
		string joint_name
		float64 position
		float64 tolerance_above
		float64 tolerance_below
		float64 weight
	PositionConstraint[] position_constraints
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string link_name
		geometry_msgs/Vector3 target_point_offset
			float64 x
			float64 y
			float64 z
		BoundingVolume constraint_region
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
		float64 weight
	OrientationConstraint[] orientation_constraints
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		geometry_msgs/Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
		string link_name
		float64 absolute_x_axis_tolerance
		float64 absolute_y_axis_tolerance
		float64 absolute_z_axis_tolerance
		uint8 parameterization
		uint8 XYZ_EULER_ANGLES=0
		uint8 ROTATION_VECTOR=1
		float64 weight
	VisibilityConstraint[] visibility_constraints
		float64 target_radius
		geometry_msgs/PoseStamped target_pose
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
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
		int32 cone_sides
		geometry_msgs/PoseStamped sensor_pose
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
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
		float64 max_view_angle
		float64 max_range_angle
		uint8 SENSOR_Z=0
		uint8 SENSOR_Y=1
		uint8 SENSOR_X=2
		uint8 sensor_view_direction
		float64 weight

# The constraints the resulting trajectory must satisfy
TrajectoryConstraints trajectory_constraints
	Constraints[] constraints
		string name
		JointConstraint[] joint_constraints
			string joint_name
			float64 position
			float64 tolerance_above
			float64 tolerance_below
			float64 weight
		PositionConstraint[] position_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string link_name
			geometry_msgs/Vector3 target_point_offset
				float64 x
				float64 y
				float64 z
			BoundingVolume constraint_region
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
			float64 weight
		OrientationConstraint[] orientation_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
			string link_name
			float64 absolute_x_axis_tolerance
			float64 absolute_y_axis_tolerance
			float64 absolute_z_axis_tolerance
			uint8 parameterization
			uint8 XYZ_EULER_ANGLES=0
			uint8 ROTATION_VECTOR=1
			float64 weight
		VisibilityConstraint[] visibility_constraints
			float64 target_radius
			geometry_msgs/PoseStamped target_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			int32 cone_sides
			geometry_msgs/PoseStamped sensor_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			float64 max_view_angle
			float64 max_range_angle
			uint8 SENSOR_Z=0
			uint8 SENSOR_Y=1
			uint8 SENSOR_X=2
			uint8 sensor_view_direction
			float64 weight

# A set of trajectories that may be used as reference or initial trajectories for (typically optimization-based) planners
# These trajectories do not override start_state or goal_constraints
GenericTrajectory[] reference_trajectories
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	trajectory_msgs/JointTrajectory[] joint_trajectory
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
	moveit_msgs/CartesianTrajectory[] cartesian_trajectory
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string tracked_frame
		CartesianTrajectoryPoint[] points
			CartesianPoint point
				geometry_msgs/Pose pose
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				geometry_msgs/Twist velocity
					Vector3  linear
						float64 x
						float64 y
						float64 z
					Vector3  angular
						float64 x
						float64 y
						float64 z
				geometry_msgs/Accel acceleration
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

# The name of the planning pipeline to use. If no name is specified,
# the configured planning pipeline will be used
string pipeline_id

# The name of the planning algorithm to use. If no name is specified,
# the default planner of the planning pipeline will be used
string planner_id

# The name of the group of joints on which this planner is operating
string group_name

# The number of times this plan is to be computed. Shortest solution
# will be reported.
int32 num_planning_attempts

# The maximum amount of time the motion planner is allowed to plan for (in seconds)
float64 allowed_planning_time

# Scaling factors for optionally reducing the maximum joint velocities and
# accelerations.  Allowed values are in (0,1].  The maximum joint velocity and
# acceleration specified in the robot model are multiplied by thier respective
# factors.  If either are outside their valid ranges (importantly, this
# includes being set to 0.0), the factor is set to the default value of 1.0
# internally (i.e., maximum joint velocity or maximum joint acceleration).
float64 max_velocity_scaling_factor
float64 max_acceleration_scaling_factor

# Maximum cartesian speed for the given end effector.
# If max_cartesian_speed <= 0 the trajectory is not modified.
# These fields require the following planning request adapter: default_planner_request_adapters/SetMaxCartesianEndEffectorSpeed
string cartesian_speed_end_effector_link
float64 max_cartesian_speed # m/s
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
- **/planning_scene**:
  - Type: moveit_msgs/msg/PlanningScene
  - **Type Details:**
# name of planning scene
string name

# full robot state
RobotState robot_state
	sensor_msgs/JointState joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] name
		float64[] position
		float64[] velocity
		float64[] effort
	sensor_msgs/MultiDOFJointState multi_dof_joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] joint_names
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
		geometry_msgs/Twist[] twist
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Wrench[] wrench
			Vector3  force
				float64 x
				float64 y
				float64 z
			Vector3  torque
				float64 x
				float64 y
				float64 z
	AttachedCollisionObject[] attached_collision_objects
		string link_name
		CollisionObject object
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		string[] touch_links
		trajectory_msgs/JointTrajectory detach_posture
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
		float64 weight
	bool is_diff

# The name of the robot model this scene is for
string robot_model_name

#additional frames for duplicating tf (with respect to the planning frame)
geometry_msgs/TransformStamped[] fixed_frame_transforms
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

#full allowed collision matrix
AllowedCollisionMatrix allowed_collision_matrix
	string[] entry_names
	AllowedCollisionEntry[] entry_values
		bool[] enabled
	string[] default_entry_names
	bool[] default_entry_values

# all link paddings
LinkPadding[] link_padding
	string link_name
	float64 padding

# all link scales
LinkScale[] link_scale
	string link_name
	float64 scale

# Attached objects, collision objects, even the octomap or collision map can have
# colors associated to them. This array specifies them.
ObjectColor[] object_colors
	string id
	std_msgs/ColorRGBA color
		float32 r
		float32 g
		float32 b
		float32 a

# the collision map
PlanningSceneWorld world
	CollisionObject[] collision_objects
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		geometry_msgs/Pose pose
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		string id
		object_recognition_msgs/ObjectType type
			string key
			string db
		shape_msgs/SolidPrimitive[] primitives
			uint8 BOX=1
			uint8 SPHERE=2
			uint8 CYLINDER=3
			uint8 CONE=4
			uint8 PRISM=5
			uint8 type
			float64[<=3] dimensions  #
			uint8 BOX_X=0
			uint8 BOX_Y=1
			uint8 BOX_Z=2
			uint8 SPHERE_RADIUS=0
			uint8 CYLINDER_HEIGHT=0
			uint8 CYLINDER_RADIUS=1
			uint8 CONE_HEIGHT=0
			uint8 CONE_RADIUS=1
			uint8 PRISM_HEIGHT=0
			geometry_msgs/Polygon polygon
				Point32[] points
					#
					#
					float32 x
					float32 y
					float32 z
		geometry_msgs/Pose[] primitive_poses
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		shape_msgs/Mesh[] meshes
			MeshTriangle[] triangles
				uint32[3] vertex_indices
			geometry_msgs/Point[] vertices
				float64 x
				float64 y
				float64 z
		geometry_msgs/Pose[] mesh_poses
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		shape_msgs/Plane[] planes
			#
			float64[4] coef
		geometry_msgs/Pose[] plane_poses
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		string[] subframe_names
		geometry_msgs/Pose[] subframe_poses
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		byte ADD=0
		byte REMOVE=1
		byte APPEND=2
		byte MOVE=3
		byte operation
	octomap_msgs/OctomapWithPose octomap
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		geometry_msgs/Pose origin
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
		octomap_msgs/Octomap octomap
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			bool binary
			string id
			float64 resolution
			int8[] data

# Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene
bool is_diff
- **/planning_scene_world**:
  - Type: moveit_msgs/msg/PlanningSceneWorld
  - **Type Details:**
# collision objects
CollisionObject[] collision_objects
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	geometry_msgs/Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	string id
	object_recognition_msgs/ObjectType type
		string key
		string db
	shape_msgs/SolidPrimitive[] primitives
		uint8 BOX=1
		uint8 SPHERE=2
		uint8 CYLINDER=3
		uint8 CONE=4
		uint8 PRISM=5
		uint8 type
		float64[<=3] dimensions  #
		uint8 BOX_X=0
		uint8 BOX_Y=1
		uint8 BOX_Z=2
		uint8 SPHERE_RADIUS=0
		uint8 CYLINDER_HEIGHT=0
		uint8 CYLINDER_RADIUS=1
		uint8 CONE_HEIGHT=0
		uint8 CONE_RADIUS=1
		uint8 PRISM_HEIGHT=0
		geometry_msgs/Polygon polygon
			Point32[] points
				#
				#
				float32 x
				float32 y
				float32 z
	geometry_msgs/Pose[] primitive_poses
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	shape_msgs/Mesh[] meshes
		MeshTriangle[] triangles
			uint32[3] vertex_indices
		geometry_msgs/Point[] vertices
			float64 x
			float64 y
			float64 z
	geometry_msgs/Pose[] mesh_poses
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	shape_msgs/Plane[] planes
		#
		float64[4] coef
	geometry_msgs/Pose[] plane_poses
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	string[] subframe_names
	geometry_msgs/Pose[] subframe_poses
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	byte ADD=0
	byte REMOVE=1
	byte APPEND=2
	byte MOVE=3
	byte operation

# The octomap that represents additional collision data
octomap_msgs/OctomapWithPose octomap
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	geometry_msgs/Pose origin
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	octomap_msgs/Octomap octomap
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		bool binary
		string id
		float64 resolution
		int8[] data
- **/recognized_object_array**:
  - Type: object_recognition_msgs/msg/RecognizedObjectArray
  - **Type Details:**
##################################################### HEADER ###########################################################
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# This message type describes a potential scene configuration: a set of objects that can explain the scene
object_recognition_msgs/RecognizedObject[] objects
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	object_recognition_msgs/ObjectType type
		string key
		string db
	float32 confidence
	sensor_msgs/PointCloud2[] point_clouds
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		uint32 height
		uint32 width
		PointField[] fields
			uint8 INT8    = 1
			uint8 UINT8   = 2
			uint8 INT16   = 3
			uint8 UINT16  = 4
			uint8 INT32   = 5
			uint8 UINT32  = 6
			uint8 FLOAT32 = 7
			uint8 FLOAT64 = 8
			string name      #
			uint32 offset    #
			uint8  datatype  #
			uint32 count     #
		bool    is_bigendian #
		uint32  point_step   #
		uint32  row_step     #
		uint8[] data         #
		bool is_dense        #
	shape_msgs/Mesh bounding_mesh
		MeshTriangle[] triangles
			uint32[3] vertex_indices
		geometry_msgs/Point[] vertices
			float64 x
			float64 y
			float64 z
	geometry_msgs/Point[] bounding_contours
		float64 x
		float64 y
		float64 z
	geometry_msgs/PoseWithCovarianceStamped pose
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		PoseWithCovariance pose
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

##################################################### SEARCH ###########################################################

# The co-occurrence matrix between the recognized objects
float32[] cooccurrence
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
- **/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback**:
  - Type: visualization_msgs/msg/InteractiveMarkerFeedback
  - **Type Details:**
# Time/frame info.
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Identifying string. Must be unique in the topic namespace.
string client_id

# Feedback message sent back from the GUI, e.g.
# when the status of an interactive marker was modified by the user.

# Specifies which interactive marker and control this message refers to
string marker_name
string control_name

# Type of the event
# KEEP_ALIVE: sent while dragging to keep up control of the marker
# MENU_SELECT: a menu entry has been selected
# BUTTON_CLICK: a button control has been clicked
# POSE_UPDATE: the pose has been changed using one of the controls
uint8 KEEP_ALIVE = 0
uint8 POSE_UPDATE = 1
uint8 MENU_SELECT = 2
uint8 BUTTON_CLICK = 3

uint8 MOUSE_DOWN = 4
uint8 MOUSE_UP = 5

uint8 event_type

# Current pose of the marker
# Note: Has to be valid for all feedback types.
geometry_msgs/Pose pose
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1

# Contains the ID of the selected menu entry
# Only valid for MENU_SELECT events.
uint32 menu_entry_id

# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point
# may contain the 3 dimensional position of the event on the
# control.  If it does, mouse_point_valid will be true.  mouse_point
# will be relative to the frame listed in the header.
geometry_msgs/Point mouse_point
	float64 x
	float64 y
	float64 z
bool mouse_point_valid
- **/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update**:
  - Type: visualization_msgs/msg/InteractiveMarkerUpdate
  - **Type Details:**
# Identifying string. Must be unique in the topic namespace
# that this server works on.
string server_id

# Sequence number.
# The client will use this to detect if it has missed an update.
uint64 seq_num

# Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.
# UPDATE: Incremental update to previous state.
#         The sequence number must be 1 higher than for
#         the previous update.
# KEEP_ALIVE: Indicates the that the server is still living.
#             The sequence number does not increase.
#             No payload data should be filled out (markers, poses, or erases).
uint8 KEEP_ALIVE = 0
uint8 UPDATE = 1

uint8 type

# Note: No guarantees on the order of processing.
#       Contents must be kept consistent by sender.

# Markers to be added or updated
InteractiveMarker[] markers
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	geometry_msgs/Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	string name
	string description
	float32 scale
	MenuEntry[] menu_entries
		#
		#
		#
		uint32 id
		uint32 parent_id
		string title
		string command
		uint8 FEEDBACK=0
		uint8 ROSRUN=1
		uint8 ROSLAUNCH=2
		uint8 command_type
	InteractiveMarkerControl[] controls
		string name
		geometry_msgs/Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
		uint8 INHERIT = 0
		uint8 FIXED = 1
		uint8 VIEW_FACING = 2
		uint8 orientation_mode
		#
		uint8 NONE = 0
		uint8 MENU = 1
		uint8 BUTTON = 2
		uint8 MOVE_AXIS = 3
		uint8 MOVE_PLANE = 4
		uint8 ROTATE_AXIS = 5
		uint8 MOVE_ROTATE = 6
		uint8 MOVE_3D = 7
		uint8 ROTATE_3D = 8
		uint8 MOVE_ROTATE_3D = 9
		uint8 interaction_mode
		bool always_visible
		#
		Marker[] markers
			#
			int32 ARROW=0
			int32 CUBE=1
			int32 SPHERE=2
			int32 CYLINDER=3
			int32 LINE_STRIP=4
			int32 LINE_LIST=5
			int32 CUBE_LIST=6
			int32 SPHERE_LIST=7
			int32 POINTS=8
			int32 TEXT_VIEW_FACING=9
			int32 MESH_RESOURCE=10
			int32 TRIANGLE_LIST=11
			int32 ADD=0
			int32 MODIFY=0
			int32 DELETE=2
			int32 DELETEALL=3
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string ns
			int32 id
			int32 type
			int32 action
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			geometry_msgs/Vector3 scale
				float64 x
				float64 y
				float64 z
			std_msgs/ColorRGBA color
				float32 r
				float32 g
				float32 b
				float32 a
			builtin_interfaces/Duration lifetime
				int32 sec
				uint32 nanosec
			bool frame_locked
			geometry_msgs/Point[] points
				float64 x
				float64 y
				float64 z
			std_msgs/ColorRGBA[] colors
				float32 r
				float32 g
				float32 b
				float32 a
			string texture_resource
			sensor_msgs/CompressedImage texture
				std_msgs/Header header #
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				                             # Header frame_id should be optical frame of camera
				                             # origin of frame should be optical center of cameara
				                             # +x should point to the right in the image
				                             # +y should point down in the image
				                             # +z should point into to plane of the image
				string format                #
				                             #   Acceptable values:
				                             #     jpeg, png, tiff
				uint8[] data                 #
			UVCoordinate[] uv_coordinates
				float32 u
				float32 v
			string text
			string mesh_resource
			MeshFile mesh_file
				string filename
				uint8[] data
			bool mesh_use_embedded_materials
		bool independent_marker_orientation
		string description

# Poses of markers that should be moved
InteractiveMarkerPose[] poses
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	geometry_msgs/Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	string name

# Names of markers to be erased
string[] erases
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
- **/trajectory_execution_event**:
  - Type: std_msgs/msg/String
  - **Type Details:**
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string data

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
- **/apply_planning_scene**:
  - Type: moveit_msgs/srv/ApplyPlanningScene
  - **Type Details:**
PlanningScene scene
	string name
	RobotState robot_state
		sensor_msgs/JointState joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] name
			float64[] position
			float64[] velocity
			float64[] effort
		sensor_msgs/MultiDOFJointState multi_dof_joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] joint_names
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
			geometry_msgs/Twist[] twist
				Vector3  linear
					float64 x
					float64 y
					float64 z
				Vector3  angular
					float64 x
					float64 y
					float64 z
			geometry_msgs/Wrench[] wrench
				Vector3  force
					float64 x
					float64 y
					float64 z
				Vector3  torque
					float64 x
					float64 y
					float64 z
		AttachedCollisionObject[] attached_collision_objects
			string link_name
			CollisionObject object
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				geometry_msgs/Pose pose
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string id
				object_recognition_msgs/ObjectType type
					string key
					string db
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Plane[] planes
					#
					float64[4] coef
				geometry_msgs/Pose[] plane_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string[] subframe_names
				geometry_msgs/Pose[] subframe_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				byte ADD=0
				byte REMOVE=1
				byte APPEND=2
				byte MOVE=3
				byte operation
			string[] touch_links
			trajectory_msgs/JointTrajectory detach_posture
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
			float64 weight
		bool is_diff
	string robot_model_name
	geometry_msgs/TransformStamped[] fixed_frame_transforms
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
	AllowedCollisionMatrix allowed_collision_matrix
		string[] entry_names
		AllowedCollisionEntry[] entry_values
			bool[] enabled
		string[] default_entry_names
		bool[] default_entry_values
	LinkPadding[] link_padding
		string link_name
		float64 padding
	LinkScale[] link_scale
		string link_name
		float64 scale
	ObjectColor[] object_colors
		string id
		std_msgs/ColorRGBA color
			float32 r
			float32 g
			float32 b
			float32 a
	PlanningSceneWorld world
		CollisionObject[] collision_objects
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		octomap_msgs/OctomapWithPose octomap
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose origin
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			octomap_msgs/Octomap octomap
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				bool binary
				string id
				float64 resolution
				int8[] data
	bool is_diff
---
bool success
- **/check_state_validity**:
  - Type: moveit_msgs/srv/GetStateValidity
  - **Type Details:**
RobotState robot_state
	sensor_msgs/JointState joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] name
		float64[] position
		float64[] velocity
		float64[] effort
	sensor_msgs/MultiDOFJointState multi_dof_joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] joint_names
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
		geometry_msgs/Twist[] twist
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Wrench[] wrench
			Vector3  force
				float64 x
				float64 y
				float64 z
			Vector3  torque
				float64 x
				float64 y
				float64 z
	AttachedCollisionObject[] attached_collision_objects
		string link_name
		CollisionObject object
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		string[] touch_links
		trajectory_msgs/JointTrajectory detach_posture
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
		float64 weight
	bool is_diff
string group_name
Constraints constraints
	string name
	JointConstraint[] joint_constraints
		string joint_name
		float64 position
		float64 tolerance_above
		float64 tolerance_below
		float64 weight
	PositionConstraint[] position_constraints
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string link_name
		geometry_msgs/Vector3 target_point_offset
			float64 x
			float64 y
			float64 z
		BoundingVolume constraint_region
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
		float64 weight
	OrientationConstraint[] orientation_constraints
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		geometry_msgs/Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
		string link_name
		float64 absolute_x_axis_tolerance
		float64 absolute_y_axis_tolerance
		float64 absolute_z_axis_tolerance
		uint8 parameterization
		uint8 XYZ_EULER_ANGLES=0
		uint8 ROTATION_VECTOR=1
		float64 weight
	VisibilityConstraint[] visibility_constraints
		float64 target_radius
		geometry_msgs/PoseStamped target_pose
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
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
		int32 cone_sides
		geometry_msgs/PoseStamped sensor_pose
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
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
		float64 max_view_angle
		float64 max_range_angle
		uint8 SENSOR_Z=0
		uint8 SENSOR_Y=1
		uint8 SENSOR_X=2
		uint8 sensor_view_direction
		float64 weight

---

bool valid
ContactInformation[] contacts
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	geometry_msgs/Point position
		float64 x
		float64 y
		float64 z
	geometry_msgs/Vector3 normal
		float64 x
		float64 y
		float64 z
	float64 depth
	string contact_body_1
	uint32 body_type_1
	string contact_body_2
	uint32 body_type_2
	uint32 ROBOT_LINK=0
	uint32 WORLD_OBJECT=1
	uint32 ROBOT_ATTACHED=2
CostSource[] cost_sources
	float64 cost_density
	geometry_msgs/Vector3 aabb_min
		float64 x
		float64 y
		float64 z
	geometry_msgs/Vector3 aabb_max
		float64 x
		float64 y
		float64 z
ConstraintEvalResult[] constraint_result
	bool result
	float64 distance
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
- **/clear_octomap**:
  - Type: std_srvs/srv/Empty
  - **Type Details:**
---
- **/compute_cartesian_path**:
  - Type: moveit_msgs/srv/GetCartesianPath
  - **Type Details:**
# Define the frame for the specified waypoints
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# The start at which to start the Cartesian path
RobotState start_state
	sensor_msgs/JointState joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] name
		float64[] position
		float64[] velocity
		float64[] effort
	sensor_msgs/MultiDOFJointState multi_dof_joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] joint_names
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
		geometry_msgs/Twist[] twist
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Wrench[] wrench
			Vector3  force
				float64 x
				float64 y
				float64 z
			Vector3  torque
				float64 x
				float64 y
				float64 z
	AttachedCollisionObject[] attached_collision_objects
		string link_name
		CollisionObject object
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		string[] touch_links
		trajectory_msgs/JointTrajectory detach_posture
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
		float64 weight
	bool is_diff

# Mandatory name of group to compute the path for
string group_name

# Optional name of IK link for which waypoints are specified.
# If not specified, the tip of the group (which is assumed to be a chain)
# is assumed to be the link
string link_name

# A sequence of waypoints to be followed by the specified link,
# while moving the specified group, such that the group moves only
# in a straight line between waypoints
geometry_msgs/Pose[] waypoints
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1

# The maximum distance (in Cartesian space) between consecutive points
# in the returned path. This must always be specified and > 0
float64 max_step

# If jump_threshold is set > 0, it acts as a scaling factor that is used to
# filter out large relative joint-space jumps in the generated Cartesian path.
# To this end, the average joint-space distance between consecutive waypoints
# is computed. If any joint-space distance is larger than this average distance
# by a factor of jump_threshold_factor, this step is considered a jump
# and the returned path is truncated before the step.
float64 jump_threshold

# If prismatic_jump_threshold or revolute_jump_threshold are set > 0, then for
# all active prismatic or revolute joints, the joint-space difference between
# consecutive waypoints is compared to the respective absolute threshold.
# If any threshold is exceeded, this step is considered a jump and the returned path
# is truncated before the step.
float64 prismatic_jump_threshold
float64 revolute_jump_threshold

# Set to true if collisions should be avoided when possible
bool avoid_collisions

# Specify additional constraints to be met by the Cartesian path
Constraints path_constraints
	string name
	JointConstraint[] joint_constraints
		string joint_name
		float64 position
		float64 tolerance_above
		float64 tolerance_below
		float64 weight
	PositionConstraint[] position_constraints
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string link_name
		geometry_msgs/Vector3 target_point_offset
			float64 x
			float64 y
			float64 z
		BoundingVolume constraint_region
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
		float64 weight
	OrientationConstraint[] orientation_constraints
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		geometry_msgs/Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
		string link_name
		float64 absolute_x_axis_tolerance
		float64 absolute_y_axis_tolerance
		float64 absolute_z_axis_tolerance
		uint8 parameterization
		uint8 XYZ_EULER_ANGLES=0
		uint8 ROTATION_VECTOR=1
		float64 weight
	VisibilityConstraint[] visibility_constraints
		float64 target_radius
		geometry_msgs/PoseStamped target_pose
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
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
		int32 cone_sides
		geometry_msgs/PoseStamped sensor_pose
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
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
		float64 max_view_angle
		float64 max_range_angle
		uint8 SENSOR_Z=0
		uint8 SENSOR_Y=1
		uint8 SENSOR_X=2
		uint8 sensor_view_direction
		float64 weight

---

# The state at which the computed path starts
RobotState start_state
	sensor_msgs/JointState joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] name
		float64[] position
		float64[] velocity
		float64[] effort
	sensor_msgs/MultiDOFJointState multi_dof_joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] joint_names
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
		geometry_msgs/Twist[] twist
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Wrench[] wrench
			Vector3  force
				float64 x
				float64 y
				float64 z
			Vector3  torque
				float64 x
				float64 y
				float64 z
	AttachedCollisionObject[] attached_collision_objects
		string link_name
		CollisionObject object
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		string[] touch_links
		trajectory_msgs/JointTrajectory detach_posture
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
		float64 weight
	bool is_diff

# The computed solution trajectory, for the desired group, in configuration space
RobotTrajectory solution
	trajectory_msgs/JointTrajectory joint_trajectory
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
	trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
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

# If the computation was incomplete, this value indicates the fraction of the path
# that was in fact computed (number of waypoints traveled through)
float64 fraction

# The error code of the computation
MoveItErrorCodes error_code
	int32 val
	int32 SUCCESS=1
	int32 FAILURE=99999
	int32 PLANNING_FAILED=-1
	int32 INVALID_MOTION_PLAN=-2
	int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
	int32 CONTROL_FAILED=-4
	int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
	int32 TIMED_OUT=-6
	int32 PREEMPTED=-7
	int32 START_STATE_IN_COLLISION=-10
	int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11
	int32 START_STATE_INVALID=-26
	int32 GOAL_IN_COLLISION=-12
	int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
	int32 GOAL_CONSTRAINTS_VIOLATED=-14
	int32 GOAL_STATE_INVALID=-27
	int32 UNRECOGNIZED_GOAL_TYPE=-28
	int32 INVALID_GROUP_NAME=-15
	int32 INVALID_GOAL_CONSTRAINTS=-16
	int32 INVALID_ROBOT_STATE=-17
	int32 INVALID_LINK_NAME=-18
	int32 INVALID_OBJECT_NAME=-19
	int32 FRAME_TRANSFORM_FAILURE=-21
	int32 COLLISION_CHECKING_UNAVAILABLE=-22
	int32 ROBOT_STATE_STALE=-23
	int32 SENSOR_INFO_STALE=-24
	int32 COMMUNICATION_FAILURE=-25
	int32 CRASH=-29
	int32 ABORT=-30
	int32 NO_IK_SOLUTION=-31
- **/compute_fk**:
  - Type: moveit_msgs/srv/GetPositionFK
  - **Type Details:**
# A service definition for a standard forward kinematics service
# The frame_id in the header message is the frame in which
# the forward kinematics poses will be returned
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# A vector of link name for which forward kinematics must be computed
string[] fk_link_names

# A robot state consisting of joint names and joint positions to be used for forward kinematics
RobotState robot_state
	sensor_msgs/JointState joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] name
		float64[] position
		float64[] velocity
		float64[] effort
	sensor_msgs/MultiDOFJointState multi_dof_joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] joint_names
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
		geometry_msgs/Twist[] twist
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Wrench[] wrench
			Vector3  force
				float64 x
				float64 y
				float64 z
			Vector3  torque
				float64 x
				float64 y
				float64 z
	AttachedCollisionObject[] attached_collision_objects
		string link_name
		CollisionObject object
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		string[] touch_links
		trajectory_msgs/JointTrajectory detach_posture
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
		float64 weight
	bool is_diff
---
# The resultant vector of PoseStamped messages that contains the (stamped) poses of the requested links
geometry_msgs/PoseStamped[] pose_stamped
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
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

# The list of link names corresponding to the poses
string[] fk_link_names

MoveItErrorCodes error_code
	int32 val
	int32 SUCCESS=1
	int32 FAILURE=99999
	int32 PLANNING_FAILED=-1
	int32 INVALID_MOTION_PLAN=-2
	int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
	int32 CONTROL_FAILED=-4
	int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
	int32 TIMED_OUT=-6
	int32 PREEMPTED=-7
	int32 START_STATE_IN_COLLISION=-10
	int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11
	int32 START_STATE_INVALID=-26
	int32 GOAL_IN_COLLISION=-12
	int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
	int32 GOAL_CONSTRAINTS_VIOLATED=-14
	int32 GOAL_STATE_INVALID=-27
	int32 UNRECOGNIZED_GOAL_TYPE=-28
	int32 INVALID_GROUP_NAME=-15
	int32 INVALID_GOAL_CONSTRAINTS=-16
	int32 INVALID_ROBOT_STATE=-17
	int32 INVALID_LINK_NAME=-18
	int32 INVALID_OBJECT_NAME=-19
	int32 FRAME_TRANSFORM_FAILURE=-21
	int32 COLLISION_CHECKING_UNAVAILABLE=-22
	int32 ROBOT_STATE_STALE=-23
	int32 SENSOR_INFO_STALE=-24
	int32 COMMUNICATION_FAILURE=-25
	int32 CRASH=-29
	int32 ABORT=-30
	int32 NO_IK_SOLUTION=-31
- **/compute_ik**:
  - Type: moveit_msgs/srv/GetPositionIK
  - **Type Details:**
# A service call to carry out an inverse kinematics computation
# The inverse kinematics request
PositionIKRequest ik_request
	string group_name
	moveit_msgs/RobotState robot_state
		sensor_msgs/JointState joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] name
			float64[] position
			float64[] velocity
			float64[] effort
		sensor_msgs/MultiDOFJointState multi_dof_joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] joint_names
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
			geometry_msgs/Twist[] twist
				Vector3  linear
					float64 x
					float64 y
					float64 z
				Vector3  angular
					float64 x
					float64 y
					float64 z
			geometry_msgs/Wrench[] wrench
				Vector3  force
					float64 x
					float64 y
					float64 z
				Vector3  torque
					float64 x
					float64 y
					float64 z
		AttachedCollisionObject[] attached_collision_objects
			string link_name
			CollisionObject object
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				geometry_msgs/Pose pose
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string id
				object_recognition_msgs/ObjectType type
					string key
					string db
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Plane[] planes
					#
					float64[4] coef
				geometry_msgs/Pose[] plane_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string[] subframe_names
				geometry_msgs/Pose[] subframe_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				byte ADD=0
				byte REMOVE=1
				byte APPEND=2
				byte MOVE=3
				byte operation
			string[] touch_links
			trajectory_msgs/JointTrajectory detach_posture
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
			float64 weight
		bool is_diff
	Constraints constraints
		string name
		JointConstraint[] joint_constraints
			string joint_name
			float64 position
			float64 tolerance_above
			float64 tolerance_below
			float64 weight
		PositionConstraint[] position_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string link_name
			geometry_msgs/Vector3 target_point_offset
				float64 x
				float64 y
				float64 z
			BoundingVolume constraint_region
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
			float64 weight
		OrientationConstraint[] orientation_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
			string link_name
			float64 absolute_x_axis_tolerance
			float64 absolute_y_axis_tolerance
			float64 absolute_z_axis_tolerance
			uint8 parameterization
			uint8 XYZ_EULER_ANGLES=0
			uint8 ROTATION_VECTOR=1
			float64 weight
		VisibilityConstraint[] visibility_constraints
			float64 target_radius
			geometry_msgs/PoseStamped target_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			int32 cone_sides
			geometry_msgs/PoseStamped sensor_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			float64 max_view_angle
			float64 max_range_angle
			uint8 SENSOR_Z=0
			uint8 SENSOR_Y=1
			uint8 SENSOR_X=2
			uint8 sensor_view_direction
			float64 weight
	bool avoid_collisions
	string ik_link_name
	geometry_msgs/PoseStamped pose_stamped
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
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
	string[] ik_link_names
	geometry_msgs/PoseStamped[] pose_stamped_vector
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
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
	builtin_interfaces/Duration timeout
		int32 sec
		uint32 nanosec

---

# The returned solution
# (in the same order as the list of joints specified in the IKRequest message)
RobotState solution
	sensor_msgs/JointState joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] name
		float64[] position
		float64[] velocity
		float64[] effort
	sensor_msgs/MultiDOFJointState multi_dof_joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] joint_names
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
		geometry_msgs/Twist[] twist
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Wrench[] wrench
			Vector3  force
				float64 x
				float64 y
				float64 z
			Vector3  torque
				float64 x
				float64 y
				float64 z
	AttachedCollisionObject[] attached_collision_objects
		string link_name
		CollisionObject object
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		string[] touch_links
		trajectory_msgs/JointTrajectory detach_posture
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
		float64 weight
	bool is_diff

MoveItErrorCodes error_code
	int32 val
	int32 SUCCESS=1
	int32 FAILURE=99999
	int32 PLANNING_FAILED=-1
	int32 INVALID_MOTION_PLAN=-2
	int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
	int32 CONTROL_FAILED=-4
	int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
	int32 TIMED_OUT=-6
	int32 PREEMPTED=-7
	int32 START_STATE_IN_COLLISION=-10
	int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11
	int32 START_STATE_INVALID=-26
	int32 GOAL_IN_COLLISION=-12
	int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
	int32 GOAL_CONSTRAINTS_VIOLATED=-14
	int32 GOAL_STATE_INVALID=-27
	int32 UNRECOGNIZED_GOAL_TYPE=-28
	int32 INVALID_GROUP_NAME=-15
	int32 INVALID_GOAL_CONSTRAINTS=-16
	int32 INVALID_ROBOT_STATE=-17
	int32 INVALID_LINK_NAME=-18
	int32 INVALID_OBJECT_NAME=-19
	int32 FRAME_TRANSFORM_FAILURE=-21
	int32 COLLISION_CHECKING_UNAVAILABLE=-22
	int32 ROBOT_STATE_STALE=-23
	int32 SENSOR_INFO_STALE=-24
	int32 COMMUNICATION_FAILURE=-25
	int32 CRASH=-29
	int32 ABORT=-30
	int32 NO_IK_SOLUTION=-31
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
- **/get_planner_params**:
  - Type: moveit_msgs/srv/GetPlannerParams
  - **Type Details:**
# Name of the planning pipeline, uses default if empty
string pipeline_id

# Name of planning config
string planner_config

# Optional name of planning group (return global defaults if empty)
string group

---

# parameters as key-value pairs
PlannerParams params
	string[] keys
	string[] values
	string[] descriptions
- **/get_planning_scene**:
  - Type: moveit_msgs/srv/GetPlanningScene
  - **Type Details:**
# Get parts of the planning scene that are of interest
# All scene components are returned if none are specified
PlanningSceneComponents components
	uint32 SCENE_SETTINGS=1
	uint32 ROBOT_STATE=2
	uint32 ROBOT_STATE_ATTACHED_OBJECTS=4
	uint32 WORLD_OBJECT_NAMES=8
	uint32 WORLD_OBJECT_GEOMETRY=16
	uint32 OCTOMAP=32
	uint32 TRANSFORMS=64
	uint32 ALLOWED_COLLISION_MATRIX=128
	uint32 LINK_PADDING_AND_SCALING=256
	uint32 OBJECT_COLORS=512
	uint32 components
---
PlanningScene scene
	string name
	RobotState robot_state
		sensor_msgs/JointState joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] name
			float64[] position
			float64[] velocity
			float64[] effort
		sensor_msgs/MultiDOFJointState multi_dof_joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] joint_names
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
			geometry_msgs/Twist[] twist
				Vector3  linear
					float64 x
					float64 y
					float64 z
				Vector3  angular
					float64 x
					float64 y
					float64 z
			geometry_msgs/Wrench[] wrench
				Vector3  force
					float64 x
					float64 y
					float64 z
				Vector3  torque
					float64 x
					float64 y
					float64 z
		AttachedCollisionObject[] attached_collision_objects
			string link_name
			CollisionObject object
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				geometry_msgs/Pose pose
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string id
				object_recognition_msgs/ObjectType type
					string key
					string db
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Plane[] planes
					#
					float64[4] coef
				geometry_msgs/Pose[] plane_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string[] subframe_names
				geometry_msgs/Pose[] subframe_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				byte ADD=0
				byte REMOVE=1
				byte APPEND=2
				byte MOVE=3
				byte operation
			string[] touch_links
			trajectory_msgs/JointTrajectory detach_posture
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
			float64 weight
		bool is_diff
	string robot_model_name
	geometry_msgs/TransformStamped[] fixed_frame_transforms
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
	AllowedCollisionMatrix allowed_collision_matrix
		string[] entry_names
		AllowedCollisionEntry[] entry_values
			bool[] enabled
		string[] default_entry_names
		bool[] default_entry_values
	LinkPadding[] link_padding
		string link_name
		float64 padding
	LinkScale[] link_scale
		string link_name
		float64 scale
	ObjectColor[] object_colors
		string id
		std_msgs/ColorRGBA color
			float32 r
			float32 g
			float32 b
			float32 a
	PlanningSceneWorld world
		CollisionObject[] collision_objects
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		octomap_msgs/OctomapWithPose octomap
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose origin
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			octomap_msgs/Octomap octomap
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				bool binary
				string id
				float64 resolution
				int8[] data
	bool is_diff
- **/interactive_marker_display_109699949054128/describe_parameters**:
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
- **/interactive_marker_display_109699949054128/get_parameter_types**:
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
- **/interactive_marker_display_109699949054128/get_parameters**:
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
- **/interactive_marker_display_109699949054128/list_parameters**:
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
- **/interactive_marker_display_109699949054128/set_parameters**:
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
- **/interactive_marker_display_109699949054128/set_parameters_atomically**:
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
- **/lifecycle_manager_localization/is_active**:
  - Type: std_srvs/srv/Trigger
  - **Type Details:**
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
- **/lifecycle_manager_localization/manage_nodes**:
  - Type: nav2_msgs/srv/ManageLifecycleNodes
  - **Type Details:**
uint8 STARTUP = 0
uint8 PAUSE = 1
uint8 RESUME = 2
uint8 RESET = 3
uint8 SHUTDOWN = 4

uint8 command
---
bool success
- **/lifecycle_manager_navigation/is_active**:
  - Type: std_srvs/srv/Trigger
  - **Type Details:**
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
- **/lifecycle_manager_navigation/manage_nodes**:
  - Type: nav2_msgs/srv/ManageLifecycleNodes
  - **Type Details:**
uint8 STARTUP = 0
uint8 PAUSE = 1
uint8 RESUME = 2
uint8 RESET = 3
uint8 SHUTDOWN = 4

uint8 command
---
bool success
- **/load_map**:
  - Type: moveit_msgs/srv/LoadMap
  - **Type Details:**
string filename
---
bool success
- **/move_group/describe_parameters**:
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
- **/move_group/get_parameter_types**:
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
- **/move_group/get_parameters**:
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
- **/move_group/list_parameters**:
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
- **/move_group/set_parameters**:
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
- **/move_group/set_parameters_atomically**:
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
- **/move_group_private_103691613681872/describe_parameters**:
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
- **/move_group_private_103691613681872/get_parameter_types**:
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
- **/move_group_private_103691613681872/get_parameters**:
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
- **/move_group_private_103691613681872/list_parameters**:
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
- **/move_group_private_103691613681872/set_parameters**:
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
- **/move_group_private_103691613681872/set_parameters_atomically**:
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
- **/moveit_simple_controller_manager/describe_parameters**:
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
- **/moveit_simple_controller_manager/get_parameter_types**:
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
- **/moveit_simple_controller_manager/get_parameters**:
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
- **/moveit_simple_controller_manager/list_parameters**:
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
- **/moveit_simple_controller_manager/set_parameters**:
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
- **/moveit_simple_controller_manager/set_parameters_atomically**:
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
- **/plan_kinematic_path**:
  - Type: moveit_msgs/srv/GetMotionPlan
  - **Type Details:**
# This service contains the definition for a request to the motion
# planner and the output it provides

MotionPlanRequest motion_plan_request
	WorkspaceParameters workspace_parameters
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		geometry_msgs/Vector3 min_corner
			float64 x
			float64 y
			float64 z
		geometry_msgs/Vector3 max_corner
			float64 x
			float64 y
			float64 z
	RobotState start_state
		sensor_msgs/JointState joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] name
			float64[] position
			float64[] velocity
			float64[] effort
		sensor_msgs/MultiDOFJointState multi_dof_joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] joint_names
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
			geometry_msgs/Twist[] twist
				Vector3  linear
					float64 x
					float64 y
					float64 z
				Vector3  angular
					float64 x
					float64 y
					float64 z
			geometry_msgs/Wrench[] wrench
				Vector3  force
					float64 x
					float64 y
					float64 z
				Vector3  torque
					float64 x
					float64 y
					float64 z
		AttachedCollisionObject[] attached_collision_objects
			string link_name
			CollisionObject object
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				geometry_msgs/Pose pose
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string id
				object_recognition_msgs/ObjectType type
					string key
					string db
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Plane[] planes
					#
					float64[4] coef
				geometry_msgs/Pose[] plane_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string[] subframe_names
				geometry_msgs/Pose[] subframe_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				byte ADD=0
				byte REMOVE=1
				byte APPEND=2
				byte MOVE=3
				byte operation
			string[] touch_links
			trajectory_msgs/JointTrajectory detach_posture
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
			float64 weight
		bool is_diff
	Constraints[] goal_constraints
		string name
		JointConstraint[] joint_constraints
			string joint_name
			float64 position
			float64 tolerance_above
			float64 tolerance_below
			float64 weight
		PositionConstraint[] position_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string link_name
			geometry_msgs/Vector3 target_point_offset
				float64 x
				float64 y
				float64 z
			BoundingVolume constraint_region
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
			float64 weight
		OrientationConstraint[] orientation_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
			string link_name
			float64 absolute_x_axis_tolerance
			float64 absolute_y_axis_tolerance
			float64 absolute_z_axis_tolerance
			uint8 parameterization
			uint8 XYZ_EULER_ANGLES=0
			uint8 ROTATION_VECTOR=1
			float64 weight
		VisibilityConstraint[] visibility_constraints
			float64 target_radius
			geometry_msgs/PoseStamped target_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			int32 cone_sides
			geometry_msgs/PoseStamped sensor_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			float64 max_view_angle
			float64 max_range_angle
			uint8 SENSOR_Z=0
			uint8 SENSOR_Y=1
			uint8 SENSOR_X=2
			uint8 sensor_view_direction
			float64 weight
	Constraints path_constraints
		string name
		JointConstraint[] joint_constraints
			string joint_name
			float64 position
			float64 tolerance_above
			float64 tolerance_below
			float64 weight
		PositionConstraint[] position_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string link_name
			geometry_msgs/Vector3 target_point_offset
				float64 x
				float64 y
				float64 z
			BoundingVolume constraint_region
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
			float64 weight
		OrientationConstraint[] orientation_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
			string link_name
			float64 absolute_x_axis_tolerance
			float64 absolute_y_axis_tolerance
			float64 absolute_z_axis_tolerance
			uint8 parameterization
			uint8 XYZ_EULER_ANGLES=0
			uint8 ROTATION_VECTOR=1
			float64 weight
		VisibilityConstraint[] visibility_constraints
			float64 target_radius
			geometry_msgs/PoseStamped target_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			int32 cone_sides
			geometry_msgs/PoseStamped sensor_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			float64 max_view_angle
			float64 max_range_angle
			uint8 SENSOR_Z=0
			uint8 SENSOR_Y=1
			uint8 SENSOR_X=2
			uint8 sensor_view_direction
			float64 weight
	TrajectoryConstraints trajectory_constraints
		Constraints[] constraints
			string name
			JointConstraint[] joint_constraints
				string joint_name
				float64 position
				float64 tolerance_above
				float64 tolerance_below
				float64 weight
			PositionConstraint[] position_constraints
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				string link_name
				geometry_msgs/Vector3 target_point_offset
					float64 x
					float64 y
					float64 z
				BoundingVolume constraint_region
					shape_msgs/SolidPrimitive[] primitives
						uint8 BOX=1
						uint8 SPHERE=2
						uint8 CYLINDER=3
						uint8 CONE=4
						uint8 PRISM=5
						uint8 type
						float64[<=3] dimensions  #
						uint8 BOX_X=0
						uint8 BOX_Y=1
						uint8 BOX_Z=2
						uint8 SPHERE_RADIUS=0
						uint8 CYLINDER_HEIGHT=0
						uint8 CYLINDER_RADIUS=1
						uint8 CONE_HEIGHT=0
						uint8 CONE_RADIUS=1
						uint8 PRISM_HEIGHT=0
						geometry_msgs/Polygon polygon
							Point32[] points
								#
								#
								float32 x
								float32 y
								float32 z
					geometry_msgs/Pose[] primitive_poses
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
					shape_msgs/Mesh[] meshes
						MeshTriangle[] triangles
							uint32[3] vertex_indices
						geometry_msgs/Point[] vertices
							float64 x
							float64 y
							float64 z
					geometry_msgs/Pose[] mesh_poses
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
				float64 weight
			OrientationConstraint[] orientation_constraints
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				geometry_msgs/Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
				string link_name
				float64 absolute_x_axis_tolerance
				float64 absolute_y_axis_tolerance
				float64 absolute_z_axis_tolerance
				uint8 parameterization
				uint8 XYZ_EULER_ANGLES=0
				uint8 ROTATION_VECTOR=1
				float64 weight
			VisibilityConstraint[] visibility_constraints
				float64 target_radius
				geometry_msgs/PoseStamped target_pose
					std_msgs/Header header
						builtin_interfaces/Time stamp
							int32 sec
							uint32 nanosec
						string frame_id
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
				int32 cone_sides
				geometry_msgs/PoseStamped sensor_pose
					std_msgs/Header header
						builtin_interfaces/Time stamp
							int32 sec
							uint32 nanosec
						string frame_id
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
				float64 max_view_angle
				float64 max_range_angle
				uint8 SENSOR_Z=0
				uint8 SENSOR_Y=1
				uint8 SENSOR_X=2
				uint8 sensor_view_direction
				float64 weight
	GenericTrajectory[] reference_trajectories
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		trajectory_msgs/JointTrajectory[] joint_trajectory
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
		moveit_msgs/CartesianTrajectory[] cartesian_trajectory
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string tracked_frame
			CartesianTrajectoryPoint[] points
				CartesianPoint point
					geometry_msgs/Pose pose
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
					geometry_msgs/Twist velocity
						Vector3  linear
							float64 x
							float64 y
							float64 z
						Vector3  angular
							float64 x
							float64 y
							float64 z
					geometry_msgs/Accel acceleration
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
	string pipeline_id
	string planner_id
	string group_name
	int32 num_planning_attempts
	float64 allowed_planning_time
	float64 max_velocity_scaling_factor
	float64 max_acceleration_scaling_factor
	string cartesian_speed_end_effector_link
	float64 max_cartesian_speed #

---

MotionPlanResponse motion_plan_response
	RobotState trajectory_start
		sensor_msgs/JointState joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] name
			float64[] position
			float64[] velocity
			float64[] effort
		sensor_msgs/MultiDOFJointState multi_dof_joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] joint_names
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
			geometry_msgs/Twist[] twist
				Vector3  linear
					float64 x
					float64 y
					float64 z
				Vector3  angular
					float64 x
					float64 y
					float64 z
			geometry_msgs/Wrench[] wrench
				Vector3  force
					float64 x
					float64 y
					float64 z
				Vector3  torque
					float64 x
					float64 y
					float64 z
		AttachedCollisionObject[] attached_collision_objects
			string link_name
			CollisionObject object
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				geometry_msgs/Pose pose
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string id
				object_recognition_msgs/ObjectType type
					string key
					string db
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Plane[] planes
					#
					float64[4] coef
				geometry_msgs/Pose[] plane_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string[] subframe_names
				geometry_msgs/Pose[] subframe_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				byte ADD=0
				byte REMOVE=1
				byte APPEND=2
				byte MOVE=3
				byte operation
			string[] touch_links
			trajectory_msgs/JointTrajectory detach_posture
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
			float64 weight
		bool is_diff
	string group_name
	RobotTrajectory trajectory
		trajectory_msgs/JointTrajectory joint_trajectory
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
		trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
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
	float64 planning_time
	MoveItErrorCodes error_code
		int32 val
		int32 SUCCESS=1
		int32 FAILURE=99999
		int32 PLANNING_FAILED=-1
		int32 INVALID_MOTION_PLAN=-2
		int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
		int32 CONTROL_FAILED=-4
		int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
		int32 TIMED_OUT=-6
		int32 PREEMPTED=-7
		int32 START_STATE_IN_COLLISION=-10
		int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11
		int32 START_STATE_INVALID=-26
		int32 GOAL_IN_COLLISION=-12
		int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
		int32 GOAL_CONSTRAINTS_VIOLATED=-14
		int32 GOAL_STATE_INVALID=-27
		int32 UNRECOGNIZED_GOAL_TYPE=-28
		int32 INVALID_GROUP_NAME=-15
		int32 INVALID_GOAL_CONSTRAINTS=-16
		int32 INVALID_ROBOT_STATE=-17
		int32 INVALID_LINK_NAME=-18
		int32 INVALID_OBJECT_NAME=-19
		int32 FRAME_TRANSFORM_FAILURE=-21
		int32 COLLISION_CHECKING_UNAVAILABLE=-22
		int32 ROBOT_STATE_STALE=-23
		int32 SENSOR_INFO_STALE=-24
		int32 COMMUNICATION_FAILURE=-25
		int32 CRASH=-29
		int32 ABORT=-30
		int32 NO_IK_SOLUTION=-31
- **/query_planner_interface**:
  - Type: moveit_msgs/srv/QueryPlannerInterfaces
  - **Type Details:**
---

# The planning instances that could be used in the benchmark
PlannerInterfaceDescription[] planner_interfaces
	string name
	string pipeline_id
	string[] planner_ids
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
- **/rviz/describe_parameters**:
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
- **/rviz/get_parameter_types**:
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
- **/rviz/get_parameters**:
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
- **/rviz/list_parameters**:
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
- **/rviz/set_parameters**:
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
- **/rviz/set_parameters_atomically**:
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
- **/rviz2_moveit/describe_parameters**:
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
- **/rviz2_moveit/get_parameter_types**:
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
- **/rviz2_moveit/get_parameters**:
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
- **/rviz2_moveit/list_parameters**:
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
- **/rviz2_moveit/set_parameters**:
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
- **/rviz2_moveit/set_parameters_atomically**:
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
- **/rviz2_moveit_private_131099787502720/describe_parameters**:
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
- **/rviz2_moveit_private_131099787502720/get_parameter_types**:
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
- **/rviz2_moveit_private_131099787502720/get_parameters**:
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
- **/rviz2_moveit_private_131099787502720/list_parameters**:
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
- **/rviz2_moveit_private_131099787502720/set_parameters**:
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
- **/rviz2_moveit_private_131099787502720/set_parameters_atomically**:
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
- **/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/get_interactive_markers**:
  - Type: visualization_msgs/srv/GetInteractiveMarkers
  - **Type Details:**
---
# Sequence number.
# Set to the sequence number of the latest update message
# at the time the server received the request.
# Clients use this to detect if any updates were missed.
uint64 sequence_number

# All interactive markers provided by the server.
InteractiveMarker[] markers
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	geometry_msgs/Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	string name
	string description
	float32 scale
	MenuEntry[] menu_entries
		#
		#
		#
		uint32 id
		uint32 parent_id
		string title
		string command
		uint8 FEEDBACK=0
		uint8 ROSRUN=1
		uint8 ROSLAUNCH=2
		uint8 command_type
	InteractiveMarkerControl[] controls
		string name
		geometry_msgs/Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
		uint8 INHERIT = 0
		uint8 FIXED = 1
		uint8 VIEW_FACING = 2
		uint8 orientation_mode
		#
		uint8 NONE = 0
		uint8 MENU = 1
		uint8 BUTTON = 2
		uint8 MOVE_AXIS = 3
		uint8 MOVE_PLANE = 4
		uint8 ROTATE_AXIS = 5
		uint8 MOVE_ROTATE = 6
		uint8 MOVE_3D = 7
		uint8 ROTATE_3D = 8
		uint8 MOVE_ROTATE_3D = 9
		uint8 interaction_mode
		bool always_visible
		#
		Marker[] markers
			#
			int32 ARROW=0
			int32 CUBE=1
			int32 SPHERE=2
			int32 CYLINDER=3
			int32 LINE_STRIP=4
			int32 LINE_LIST=5
			int32 CUBE_LIST=6
			int32 SPHERE_LIST=7
			int32 POINTS=8
			int32 TEXT_VIEW_FACING=9
			int32 MESH_RESOURCE=10
			int32 TRIANGLE_LIST=11
			int32 ADD=0
			int32 MODIFY=0
			int32 DELETE=2
			int32 DELETEALL=3
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string ns
			int32 id
			int32 type
			int32 action
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			geometry_msgs/Vector3 scale
				float64 x
				float64 y
				float64 z
			std_msgs/ColorRGBA color
				float32 r
				float32 g
				float32 b
				float32 a
			builtin_interfaces/Duration lifetime
				int32 sec
				uint32 nanosec
			bool frame_locked
			geometry_msgs/Point[] points
				float64 x
				float64 y
				float64 z
			std_msgs/ColorRGBA[] colors
				float32 r
				float32 g
				float32 b
				float32 a
			string texture_resource
			sensor_msgs/CompressedImage texture
				std_msgs/Header header #
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				                             # Header frame_id should be optical frame of camera
				                             # origin of frame should be optical center of cameara
				                             # +x should point to the right in the image
				                             # +y should point down in the image
				                             # +z should point into to plane of the image
				string format                #
				                             #   Acceptable values:
				                             #     jpeg, png, tiff
				uint8[] data                 #
			UVCoordinate[] uv_coordinates
				float32 u
				float32 v
			string text
			string mesh_resource
			MeshFile mesh_file
				string filename
				uint8[] data
			bool mesh_use_embedded_materials
		bool independent_marker_orientation
		string description
- **/rviz_navigation_dialog_action_client/describe_parameters**:
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
- **/rviz_navigation_dialog_action_client/get_parameter_types**:
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
- **/rviz_navigation_dialog_action_client/get_parameters**:
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
- **/rviz_navigation_dialog_action_client/list_parameters**:
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
- **/rviz_navigation_dialog_action_client/set_parameters**:
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
- **/rviz_navigation_dialog_action_client/set_parameters_atomically**:
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
- **/save_map**:
  - Type: moveit_msgs/srv/SaveMap
  - **Type Details:**
string filename

---

bool success
- **/set_planner_params**:
  - Type: moveit_msgs/srv/SetPlannerParams
  - **Type Details:**
# Name of the planning pipeline, uses default if empty
string pipeline_id

# Name of planning config
string planner_config

# Optional name of planning group (set global defaults if empty)
string group

# parameters as key-value pairs
PlannerParams params
	string[] keys
	string[] values
	string[] descriptions

# replace params or augment existing ones?
bool replace

---
- **/slam_toolbox/clear_changes**:
  - Type: slam_toolbox/srv/Clear
  - **Type Details:**
---
- **/slam_toolbox/clear_queue**:
  - Type: slam_toolbox/srv/ClearQueue
  - **Type Details:**
---
bool status
- **/slam_toolbox/describe_parameters**:
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
- **/slam_toolbox/deserialize_map**:
  - Type: slam_toolbox/srv/DeserializePoseGraph
  - **Type Details:**
int8 UNSET = 0
int8 START_AT_FIRST_NODE = 1
int8 START_AT_GIVEN_POSE = 2
int8 LOCALIZE_AT_POSE = 3

# inital_pose should be Map -> base_frame (parameter, generally base_link)
#

string filename
int8 match_type
geometry_msgs/Pose2D initial_pose
	float64 x
	float64 y
	float64 theta
---
- **/slam_toolbox/dynamic_map**:
  - Type: nav_msgs/srv/GetMap
  - **Type Details:**
# Get the map as a nav_msgs/OccupancyGrid
---
# The current map hosted by this map service.
OccupancyGrid map
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	MapMetaData info
		builtin_interfaces/Time map_load_time
			int32 sec
			uint32 nanosec
		float32 resolution
		uint32 width
		uint32 height
		geometry_msgs/Pose origin
			Point position
				float64 x
				float64 y
				float64 z
			Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
	int8[] data
- **/slam_toolbox/get_interactive_markers**:
  - Type: visualization_msgs/srv/GetInteractiveMarkers
  - **Type Details:**
---
# Sequence number.
# Set to the sequence number of the latest update message
# at the time the server received the request.
# Clients use this to detect if any updates were missed.
uint64 sequence_number

# All interactive markers provided by the server.
InteractiveMarker[] markers
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	geometry_msgs/Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	string name
	string description
	float32 scale
	MenuEntry[] menu_entries
		#
		#
		#
		uint32 id
		uint32 parent_id
		string title
		string command
		uint8 FEEDBACK=0
		uint8 ROSRUN=1
		uint8 ROSLAUNCH=2
		uint8 command_type
	InteractiveMarkerControl[] controls
		string name
		geometry_msgs/Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
		uint8 INHERIT = 0
		uint8 FIXED = 1
		uint8 VIEW_FACING = 2
		uint8 orientation_mode
		#
		uint8 NONE = 0
		uint8 MENU = 1
		uint8 BUTTON = 2
		uint8 MOVE_AXIS = 3
		uint8 MOVE_PLANE = 4
		uint8 ROTATE_AXIS = 5
		uint8 MOVE_ROTATE = 6
		uint8 MOVE_3D = 7
		uint8 ROTATE_3D = 8
		uint8 MOVE_ROTATE_3D = 9
		uint8 interaction_mode
		bool always_visible
		#
		Marker[] markers
			#
			int32 ARROW=0
			int32 CUBE=1
			int32 SPHERE=2
			int32 CYLINDER=3
			int32 LINE_STRIP=4
			int32 LINE_LIST=5
			int32 CUBE_LIST=6
			int32 SPHERE_LIST=7
			int32 POINTS=8
			int32 TEXT_VIEW_FACING=9
			int32 MESH_RESOURCE=10
			int32 TRIANGLE_LIST=11
			int32 ADD=0
			int32 MODIFY=0
			int32 DELETE=2
			int32 DELETEALL=3
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string ns
			int32 id
			int32 type
			int32 action
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			geometry_msgs/Vector3 scale
				float64 x
				float64 y
				float64 z
			std_msgs/ColorRGBA color
				float32 r
				float32 g
				float32 b
				float32 a
			builtin_interfaces/Duration lifetime
				int32 sec
				uint32 nanosec
			bool frame_locked
			geometry_msgs/Point[] points
				float64 x
				float64 y
				float64 z
			std_msgs/ColorRGBA[] colors
				float32 r
				float32 g
				float32 b
				float32 a
			string texture_resource
			sensor_msgs/CompressedImage texture
				std_msgs/Header header #
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				                             # Header frame_id should be optical frame of camera
				                             # origin of frame should be optical center of cameara
				                             # +x should point to the right in the image
				                             # +y should point down in the image
				                             # +z should point into to plane of the image
				string format                #
				                             #   Acceptable values:
				                             #     jpeg, png, tiff
				uint8[] data                 #
			UVCoordinate[] uv_coordinates
				float32 u
				float32 v
			string text
			string mesh_resource
			MeshFile mesh_file
				string filename
				uint8[] data
			bool mesh_use_embedded_materials
		bool independent_marker_orientation
		string description
- **/slam_toolbox/get_parameter_types**:
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
- **/slam_toolbox/get_parameters**:
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
- **/slam_toolbox/list_parameters**:
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
- **/slam_toolbox/manual_loop_closure**:
  - Type: slam_toolbox/srv/LoopClosure
  - **Type Details:**
---
- **/slam_toolbox/pause_new_measurements**:
  - Type: slam_toolbox/srv/Pause
  - **Type Details:**
# trigger pause toggle

---
bool status
- **/slam_toolbox/save_map**:
  - Type: slam_toolbox/srv/SaveMap
  - **Type Details:**
std_msgs/String name
	string data
---
# Result code defintions
uint8 RESULT_SUCCESS=0
uint8 RESULT_NO_MAP_RECEIEVD=1
uint8 RESULT_UNDEFINED_FAILURE=255

uint8 result
- **/slam_toolbox/serialize_map**:
  - Type: slam_toolbox/srv/SerializePoseGraph
  - **Type Details:**
string filename
---
# Result code defintions
uint8 RESULT_SUCCESS=0
uint8 RESULT_FAILED_TO_WRITE_FILE=255

uint8 result
- **/slam_toolbox/set_parameters**:
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
- **/slam_toolbox/set_parameters_atomically**:
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
- **/slam_toolbox/toggle_interactive_mode**:
  - Type: slam_toolbox/srv/ToggleInteractive
  - **Type Details:**
---
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
- **/execute_trajectory**:
  - Type: moveit_msgs/action/ExecuteTrajectory
  - **Type Details:**
# The trajectory to execute
RobotTrajectory trajectory
	trajectory_msgs/JointTrajectory joint_trajectory
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
	trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
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

---

# Error code - encodes the overall reason for failure
MoveItErrorCodes error_code
	int32 val
	int32 SUCCESS=1
	int32 FAILURE=99999
	int32 PLANNING_FAILED=-1
	int32 INVALID_MOTION_PLAN=-2
	int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
	int32 CONTROL_FAILED=-4
	int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
	int32 TIMED_OUT=-6
	int32 PREEMPTED=-7
	int32 START_STATE_IN_COLLISION=-10
	int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11
	int32 START_STATE_INVALID=-26
	int32 GOAL_IN_COLLISION=-12
	int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
	int32 GOAL_CONSTRAINTS_VIOLATED=-14
	int32 GOAL_STATE_INVALID=-27
	int32 UNRECOGNIZED_GOAL_TYPE=-28
	int32 INVALID_GROUP_NAME=-15
	int32 INVALID_GOAL_CONSTRAINTS=-16
	int32 INVALID_ROBOT_STATE=-17
	int32 INVALID_LINK_NAME=-18
	int32 INVALID_OBJECT_NAME=-19
	int32 FRAME_TRANSFORM_FAILURE=-21
	int32 COLLISION_CHECKING_UNAVAILABLE=-22
	int32 ROBOT_STATE_STALE=-23
	int32 SENSOR_INFO_STALE=-24
	int32 COMMUNICATION_FAILURE=-25
	int32 CRASH=-29
	int32 ABORT=-30
	int32 NO_IK_SOLUTION=-31

---

# The internal state that the move group action currently is in
string state
- **/follow_waypoints**:
  - Type: nav2_msgs/action/FollowWaypoints
  - **Type Details:**
#goal definition
geometry_msgs/PoseStamped[] poses
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
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
---
#result definition
int32[] missed_waypoints
---
#feedback definition
uint32 current_waypoint
- **/joint_trajectory_controller/follow_joint_trajectory**:
  - Type: control_msgs/action/FollowJointTrajectory
  - **Type Details:**
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
- **/move_action**:
  - Type: moveit_msgs/action/MoveGroup
  - **Type Details:**
# Motion planning request to pass to planner
MotionPlanRequest request
	WorkspaceParameters workspace_parameters
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		geometry_msgs/Vector3 min_corner
			float64 x
			float64 y
			float64 z
		geometry_msgs/Vector3 max_corner
			float64 x
			float64 y
			float64 z
	RobotState start_state
		sensor_msgs/JointState joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] name
			float64[] position
			float64[] velocity
			float64[] effort
		sensor_msgs/MultiDOFJointState multi_dof_joint_state
			#
			#
			#
			#
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string[] joint_names
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
			geometry_msgs/Twist[] twist
				Vector3  linear
					float64 x
					float64 y
					float64 z
				Vector3  angular
					float64 x
					float64 y
					float64 z
			geometry_msgs/Wrench[] wrench
				Vector3  force
					float64 x
					float64 y
					float64 z
				Vector3  torque
					float64 x
					float64 y
					float64 z
		AttachedCollisionObject[] attached_collision_objects
			string link_name
			CollisionObject object
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				geometry_msgs/Pose pose
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string id
				object_recognition_msgs/ObjectType type
					string key
					string db
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Plane[] planes
					#
					float64[4] coef
				geometry_msgs/Pose[] plane_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string[] subframe_names
				geometry_msgs/Pose[] subframe_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				byte ADD=0
				byte REMOVE=1
				byte APPEND=2
				byte MOVE=3
				byte operation
			string[] touch_links
			trajectory_msgs/JointTrajectory detach_posture
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
			float64 weight
		bool is_diff
	Constraints[] goal_constraints
		string name
		JointConstraint[] joint_constraints
			string joint_name
			float64 position
			float64 tolerance_above
			float64 tolerance_below
			float64 weight
		PositionConstraint[] position_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string link_name
			geometry_msgs/Vector3 target_point_offset
				float64 x
				float64 y
				float64 z
			BoundingVolume constraint_region
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
			float64 weight
		OrientationConstraint[] orientation_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
			string link_name
			float64 absolute_x_axis_tolerance
			float64 absolute_y_axis_tolerance
			float64 absolute_z_axis_tolerance
			uint8 parameterization
			uint8 XYZ_EULER_ANGLES=0
			uint8 ROTATION_VECTOR=1
			float64 weight
		VisibilityConstraint[] visibility_constraints
			float64 target_radius
			geometry_msgs/PoseStamped target_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			int32 cone_sides
			geometry_msgs/PoseStamped sensor_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			float64 max_view_angle
			float64 max_range_angle
			uint8 SENSOR_Z=0
			uint8 SENSOR_Y=1
			uint8 SENSOR_X=2
			uint8 sensor_view_direction
			float64 weight
	Constraints path_constraints
		string name
		JointConstraint[] joint_constraints
			string joint_name
			float64 position
			float64 tolerance_above
			float64 tolerance_below
			float64 weight
		PositionConstraint[] position_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string link_name
			geometry_msgs/Vector3 target_point_offset
				float64 x
				float64 y
				float64 z
			BoundingVolume constraint_region
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
			float64 weight
		OrientationConstraint[] orientation_constraints
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Quaternion orientation
				float64 x 0
				float64 y 0
				float64 z 0
				float64 w 1
			string link_name
			float64 absolute_x_axis_tolerance
			float64 absolute_y_axis_tolerance
			float64 absolute_z_axis_tolerance
			uint8 parameterization
			uint8 XYZ_EULER_ANGLES=0
			uint8 ROTATION_VECTOR=1
			float64 weight
		VisibilityConstraint[] visibility_constraints
			float64 target_radius
			geometry_msgs/PoseStamped target_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			int32 cone_sides
			geometry_msgs/PoseStamped sensor_pose
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
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
			float64 max_view_angle
			float64 max_range_angle
			uint8 SENSOR_Z=0
			uint8 SENSOR_Y=1
			uint8 SENSOR_X=2
			uint8 sensor_view_direction
			float64 weight
	TrajectoryConstraints trajectory_constraints
		Constraints[] constraints
			string name
			JointConstraint[] joint_constraints
				string joint_name
				float64 position
				float64 tolerance_above
				float64 tolerance_below
				float64 weight
			PositionConstraint[] position_constraints
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				string link_name
				geometry_msgs/Vector3 target_point_offset
					float64 x
					float64 y
					float64 z
				BoundingVolume constraint_region
					shape_msgs/SolidPrimitive[] primitives
						uint8 BOX=1
						uint8 SPHERE=2
						uint8 CYLINDER=3
						uint8 CONE=4
						uint8 PRISM=5
						uint8 type
						float64[<=3] dimensions  #
						uint8 BOX_X=0
						uint8 BOX_Y=1
						uint8 BOX_Z=2
						uint8 SPHERE_RADIUS=0
						uint8 CYLINDER_HEIGHT=0
						uint8 CYLINDER_RADIUS=1
						uint8 CONE_HEIGHT=0
						uint8 CONE_RADIUS=1
						uint8 PRISM_HEIGHT=0
						geometry_msgs/Polygon polygon
							Point32[] points
								#
								#
								float32 x
								float32 y
								float32 z
					geometry_msgs/Pose[] primitive_poses
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
					shape_msgs/Mesh[] meshes
						MeshTriangle[] triangles
							uint32[3] vertex_indices
						geometry_msgs/Point[] vertices
							float64 x
							float64 y
							float64 z
					geometry_msgs/Pose[] mesh_poses
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
				float64 weight
			OrientationConstraint[] orientation_constraints
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				geometry_msgs/Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
				string link_name
				float64 absolute_x_axis_tolerance
				float64 absolute_y_axis_tolerance
				float64 absolute_z_axis_tolerance
				uint8 parameterization
				uint8 XYZ_EULER_ANGLES=0
				uint8 ROTATION_VECTOR=1
				float64 weight
			VisibilityConstraint[] visibility_constraints
				float64 target_radius
				geometry_msgs/PoseStamped target_pose
					std_msgs/Header header
						builtin_interfaces/Time stamp
							int32 sec
							uint32 nanosec
						string frame_id
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
				int32 cone_sides
				geometry_msgs/PoseStamped sensor_pose
					std_msgs/Header header
						builtin_interfaces/Time stamp
							int32 sec
							uint32 nanosec
						string frame_id
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
				float64 max_view_angle
				float64 max_range_angle
				uint8 SENSOR_Z=0
				uint8 SENSOR_Y=1
				uint8 SENSOR_X=2
				uint8 sensor_view_direction
				float64 weight
	GenericTrajectory[] reference_trajectories
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		trajectory_msgs/JointTrajectory[] joint_trajectory
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
		moveit_msgs/CartesianTrajectory[] cartesian_trajectory
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			string tracked_frame
			CartesianTrajectoryPoint[] points
				CartesianPoint point
					geometry_msgs/Pose pose
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
					geometry_msgs/Twist velocity
						Vector3  linear
							float64 x
							float64 y
							float64 z
						Vector3  angular
							float64 x
							float64 y
							float64 z
					geometry_msgs/Accel acceleration
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
	string pipeline_id
	string planner_id
	string group_name
	int32 num_planning_attempts
	float64 allowed_planning_time
	float64 max_velocity_scaling_factor
	float64 max_acceleration_scaling_factor
	string cartesian_speed_end_effector_link
	float64 max_cartesian_speed #

# Planning options
PlanningOptions planning_options
	PlanningScene planning_scene_diff
		string name
		RobotState robot_state
			sensor_msgs/JointState joint_state
				#
				#
				#
				#
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				string[] name
				float64[] position
				float64[] velocity
				float64[] effort
			sensor_msgs/MultiDOFJointState multi_dof_joint_state
				#
				#
				#
				#
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				string[] joint_names
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
				geometry_msgs/Twist[] twist
					Vector3  linear
						float64 x
						float64 y
						float64 z
					Vector3  angular
						float64 x
						float64 y
						float64 z
				geometry_msgs/Wrench[] wrench
					Vector3  force
						float64 x
						float64 y
						float64 z
					Vector3  torque
						float64 x
						float64 y
						float64 z
			AttachedCollisionObject[] attached_collision_objects
				string link_name
				CollisionObject object
					std_msgs/Header header
						builtin_interfaces/Time stamp
							int32 sec
							uint32 nanosec
						string frame_id
					geometry_msgs/Pose pose
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
					string id
					object_recognition_msgs/ObjectType type
						string key
						string db
					shape_msgs/SolidPrimitive[] primitives
						uint8 BOX=1
						uint8 SPHERE=2
						uint8 CYLINDER=3
						uint8 CONE=4
						uint8 PRISM=5
						uint8 type
						float64[<=3] dimensions  #
						uint8 BOX_X=0
						uint8 BOX_Y=1
						uint8 BOX_Z=2
						uint8 SPHERE_RADIUS=0
						uint8 CYLINDER_HEIGHT=0
						uint8 CYLINDER_RADIUS=1
						uint8 CONE_HEIGHT=0
						uint8 CONE_RADIUS=1
						uint8 PRISM_HEIGHT=0
						geometry_msgs/Polygon polygon
							Point32[] points
								#
								#
								float32 x
								float32 y
								float32 z
					geometry_msgs/Pose[] primitive_poses
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
					shape_msgs/Mesh[] meshes
						MeshTriangle[] triangles
							uint32[3] vertex_indices
						geometry_msgs/Point[] vertices
							float64 x
							float64 y
							float64 z
					geometry_msgs/Pose[] mesh_poses
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
					shape_msgs/Plane[] planes
						#
						float64[4] coef
					geometry_msgs/Pose[] plane_poses
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
					string[] subframe_names
					geometry_msgs/Pose[] subframe_poses
						Point position
							float64 x
							float64 y
							float64 z
						Quaternion orientation
							float64 x 0
							float64 y 0
							float64 z 0
							float64 w 1
					byte ADD=0
					byte REMOVE=1
					byte APPEND=2
					byte MOVE=3
					byte operation
				string[] touch_links
				trajectory_msgs/JointTrajectory detach_posture
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
				float64 weight
			bool is_diff
		string robot_model_name
		geometry_msgs/TransformStamped[] fixed_frame_transforms
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
		AllowedCollisionMatrix allowed_collision_matrix
			string[] entry_names
			AllowedCollisionEntry[] entry_values
				bool[] enabled
			string[] default_entry_names
			bool[] default_entry_values
		LinkPadding[] link_padding
			string link_name
			float64 padding
		LinkScale[] link_scale
			string link_name
			float64 scale
		ObjectColor[] object_colors
			string id
			std_msgs/ColorRGBA color
				float32 r
				float32 g
				float32 b
				float32 a
		PlanningSceneWorld world
			CollisionObject[] collision_objects
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				geometry_msgs/Pose pose
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string id
				object_recognition_msgs/ObjectType type
					string key
					string db
				shape_msgs/SolidPrimitive[] primitives
					uint8 BOX=1
					uint8 SPHERE=2
					uint8 CYLINDER=3
					uint8 CONE=4
					uint8 PRISM=5
					uint8 type
					float64[<=3] dimensions  #
					uint8 BOX_X=0
					uint8 BOX_Y=1
					uint8 BOX_Z=2
					uint8 SPHERE_RADIUS=0
					uint8 CYLINDER_HEIGHT=0
					uint8 CYLINDER_RADIUS=1
					uint8 CONE_HEIGHT=0
					uint8 CONE_RADIUS=1
					uint8 PRISM_HEIGHT=0
					geometry_msgs/Polygon polygon
						Point32[] points
							#
							#
							float32 x
							float32 y
							float32 z
				geometry_msgs/Pose[] primitive_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Mesh[] meshes
					MeshTriangle[] triangles
						uint32[3] vertex_indices
					geometry_msgs/Point[] vertices
						float64 x
						float64 y
						float64 z
				geometry_msgs/Pose[] mesh_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				shape_msgs/Plane[] planes
					#
					float64[4] coef
				geometry_msgs/Pose[] plane_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				string[] subframe_names
				geometry_msgs/Pose[] subframe_poses
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				byte ADD=0
				byte REMOVE=1
				byte APPEND=2
				byte MOVE=3
				byte operation
			octomap_msgs/OctomapWithPose octomap
				std_msgs/Header header
					builtin_interfaces/Time stamp
						int32 sec
						uint32 nanosec
					string frame_id
				geometry_msgs/Pose origin
					Point position
						float64 x
						float64 y
						float64 z
					Quaternion orientation
						float64 x 0
						float64 y 0
						float64 z 0
						float64 w 1
				octomap_msgs/Octomap octomap
					std_msgs/Header header
						builtin_interfaces/Time stamp
							int32 sec
							uint32 nanosec
						string frame_id
					bool binary
					string id
					float64 resolution
					int8[] data
		bool is_diff
	bool plan_only
	bool look_around
	int32 look_around_attempts
	float64 max_safe_execution_cost
	bool replan
	int32 replan_attempts
	float64 replan_delay

---

# An error code reflecting what went wrong
MoveItErrorCodes error_code
	int32 val
	int32 SUCCESS=1
	int32 FAILURE=99999
	int32 PLANNING_FAILED=-1
	int32 INVALID_MOTION_PLAN=-2
	int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
	int32 CONTROL_FAILED=-4
	int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
	int32 TIMED_OUT=-6
	int32 PREEMPTED=-7
	int32 START_STATE_IN_COLLISION=-10
	int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11
	int32 START_STATE_INVALID=-26
	int32 GOAL_IN_COLLISION=-12
	int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
	int32 GOAL_CONSTRAINTS_VIOLATED=-14
	int32 GOAL_STATE_INVALID=-27
	int32 UNRECOGNIZED_GOAL_TYPE=-28
	int32 INVALID_GROUP_NAME=-15
	int32 INVALID_GOAL_CONSTRAINTS=-16
	int32 INVALID_ROBOT_STATE=-17
	int32 INVALID_LINK_NAME=-18
	int32 INVALID_OBJECT_NAME=-19
	int32 FRAME_TRANSFORM_FAILURE=-21
	int32 COLLISION_CHECKING_UNAVAILABLE=-22
	int32 ROBOT_STATE_STALE=-23
	int32 SENSOR_INFO_STALE=-24
	int32 COMMUNICATION_FAILURE=-25
	int32 CRASH=-29
	int32 ABORT=-30
	int32 NO_IK_SOLUTION=-31

# The full starting state of the robot at the start of the trajectory
moveit_msgs/RobotState trajectory_start
	sensor_msgs/JointState joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] name
		float64[] position
		float64[] velocity
		float64[] effort
	sensor_msgs/MultiDOFJointState multi_dof_joint_state
		#
		#
		#
		#
		std_msgs/Header header
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		string[] joint_names
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
		geometry_msgs/Twist[] twist
			Vector3  linear
				float64 x
				float64 y
				float64 z
			Vector3  angular
				float64 x
				float64 y
				float64 z
		geometry_msgs/Wrench[] wrench
			Vector3  force
				float64 x
				float64 y
				float64 z
			Vector3  torque
				float64 x
				float64 y
				float64 z
	AttachedCollisionObject[] attached_collision_objects
		string link_name
		CollisionObject object
			std_msgs/Header header
				builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
				string frame_id
			geometry_msgs/Pose pose
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string id
			object_recognition_msgs/ObjectType type
				string key
				string db
			shape_msgs/SolidPrimitive[] primitives
				uint8 BOX=1
				uint8 SPHERE=2
				uint8 CYLINDER=3
				uint8 CONE=4
				uint8 PRISM=5
				uint8 type
				float64[<=3] dimensions  #
				uint8 BOX_X=0
				uint8 BOX_Y=1
				uint8 BOX_Z=2
				uint8 SPHERE_RADIUS=0
				uint8 CYLINDER_HEIGHT=0
				uint8 CYLINDER_RADIUS=1
				uint8 CONE_HEIGHT=0
				uint8 CONE_RADIUS=1
				uint8 PRISM_HEIGHT=0
				geometry_msgs/Polygon polygon
					Point32[] points
						#
						#
						float32 x
						float32 y
						float32 z
			geometry_msgs/Pose[] primitive_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Mesh[] meshes
				MeshTriangle[] triangles
					uint32[3] vertex_indices
				geometry_msgs/Point[] vertices
					float64 x
					float64 y
					float64 z
			geometry_msgs/Pose[] mesh_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			shape_msgs/Plane[] planes
				#
				float64[4] coef
			geometry_msgs/Pose[] plane_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			string[] subframe_names
			geometry_msgs/Pose[] subframe_poses
				Point position
					float64 x
					float64 y
					float64 z
				Quaternion orientation
					float64 x 0
					float64 y 0
					float64 z 0
					float64 w 1
			byte ADD=0
			byte REMOVE=1
			byte APPEND=2
			byte MOVE=3
			byte operation
		string[] touch_links
		trajectory_msgs/JointTrajectory detach_posture
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
		float64 weight
	bool is_diff

# The trajectory that moved group produced for execution
moveit_msgs/RobotTrajectory planned_trajectory
	trajectory_msgs/JointTrajectory joint_trajectory
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
	trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
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

# The trace of the trajectory recorded during execution
moveit_msgs/RobotTrajectory executed_trajectory
	trajectory_msgs/JointTrajectory joint_trajectory
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
	trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
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

# The amount of time it took to complete the motion plan
float64 planning_time

---

# The internal state that the move group action currently is in
string state
- **/navigate_through_poses**:
  - Type: nav2_msgs/action/NavigateThroughPoses
  - **Type Details:**
#goal definition
geometry_msgs/PoseStamped[] poses
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
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
string behavior_tree
---
#result definition
std_msgs/Empty result
---
#feedback definition
geometry_msgs/PoseStamped current_pose
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
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
builtin_interfaces/Duration navigation_time
	int32 sec
	uint32 nanosec
builtin_interfaces/Duration estimated_time_remaining
	int32 sec
	uint32 nanosec
int16 number_of_recoveries
float32 distance_remaining
int16 number_of_poses_remaining
- **/navigate_to_pose**:
  - Type: nav2_msgs/action/NavigateToPose
  - **Type Details:**
#goal definition
geometry_msgs/PoseStamped pose
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
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
string behavior_tree
---
#result definition
std_msgs/Empty result
---
#feedback definition
geometry_msgs/PoseStamped current_pose
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
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
builtin_interfaces/Duration navigation_time
	int32 sec
	uint32 nanosec
builtin_interfaces/Duration estimated_time_remaining
	int32 sec
	uint32 nanosec
int16 number_of_recoveries
float32 distance_remaining
- **/scaled_joint_trajectory_controller/follow_joint_trajectory**:
  - Type: control_msgs/action/FollowJointTrajectory
  - **Type Details:**
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
