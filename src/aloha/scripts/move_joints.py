from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

def main():
    # Create the node first, then start the robot, then create the bot
    node = create_interbotix_global_node('aloha')
    print('Created node')
    robot_startup(node)
    print('robot startup successful')

    # 'vx300s' is the model. 
    # If using the Aloha setup, ensure the 'robot_name' matches your launch file (e.g., 'master_right')
    robot = InterbotixManipulatorXS(
        robot_model="aloha_vx300s", 
        robot_name="aloha_vx300s",
        node=node,
    )

    # Disabling torque allows for manual movement
    print("Disabling torque. You can now move the robot freely.")
    robot.core.robot_set_motor_registers("group", "all", "Torque_Enable", 0)

    input("Press Enter to re-enable torque and hold position...")

    # Re-enabling torque will lock the robot in its current position
    robot.core.robot_set_motor_registers("group", "all", "Torque_Enable", 1)
    print("Torque enabled.")
    
    robot_shutdown(node)

if __name__ == "__main__":
    main()