# Start an virtual RViz session

This launches a virtual robot for you to give an idea of basic control.

To get started, open a terminal and type:
    ros2 launch interbotix_xsarm_control xsarm_control.launch.py use_sim:=true robot_model:=aloha_vx300s

Then change to this directory and type:

    python3 ee_cartesian_trajectory.py --robot_model aloha_vx300s

