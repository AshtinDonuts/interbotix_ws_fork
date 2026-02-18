# Motor specifications for each joint
# This message contains torque constants and current units loaded from motor specs

std_msgs/Header header          # Standard ROS header with timestamp and frame_id
string group_name               # Name of the joint group (e.g., "arm")
string[] joint_names            # Joint names corresponding to the arrays
float32[] torque_constants      # Torque constants for each joint [Nm/A]
float32[] current_units         # Current units for each joint
