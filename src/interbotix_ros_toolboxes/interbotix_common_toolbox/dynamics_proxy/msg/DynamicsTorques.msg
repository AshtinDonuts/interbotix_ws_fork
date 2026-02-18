# Dynamics torques computed for the current joint configuration
# This message is published continuously while dynamics computation is enabled

std_msgs/Header header          # Standard ROS header with timestamp and frame_id
string group_name               # Name of the joint group (e.g., "arm")
float64[] torques               # Dynamics torques for each joint [Nm]
float64[] joint_positions       # Joint positions at time of computation [rad]
float64[] joint_velocities      # Joint velocities at time of computation [rad/s]
float64[] kinetic_friction_torques  # Kinetic friction torque contribution for each joint [Nm]
float64[] static_friction_torques   # Static friction torque contribution for each joint [Nm]
float64[] dither_speeds         # Dither speed thresholds for each joint [rad/s]
float64[] no_load_currents        # No-load current for friction compensation for each joint [A]