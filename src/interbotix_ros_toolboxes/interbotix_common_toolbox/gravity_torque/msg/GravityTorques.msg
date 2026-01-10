# Gravity compensation torques computed for the current joint configuration
# This message is published continuously while gravity compensation is enabled

std_msgs/Header header          # Standard ROS header with timestamp and frame_id
string group_name               # Name of the joint group (e.g., "arm")
float64[] torques               # Gravity compensation torques for each joint [Nm]
float64[] joint_positions       # Joint positions at time of computation [rad]
float64[] joint_velocities      # Joint velocities at time of computation [rad/s]