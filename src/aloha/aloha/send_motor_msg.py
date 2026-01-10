#!/usr/bin/env python3
"""
Script to directly send a JointSingleCommand message to a robot's gripper.

Usage:
    python send_motor_msg.py --robot_name follower_solo --position 1.2
    python send_motor_msg.py --robot_name follower_right --position 0.75 --normalized
    
Command type: Position command (in radians when operating mode is 'current_based_position')
Range for follower gripper: 0.75 (closed) to 1.74 (open) radians
"""

import argparse
import time

import rclpy
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    InterbotixRobotNode,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand


def setup_gripper(bot: InterbotixManipulatorXS):
    """
    Setup gripper with proper operating mode and parameters.
    This mimics the setup done in teleop.py opening_ceremony().
    """
    print(f"Setting up gripper for {bot.robot_name}...")
    
    # Reboot gripper motors
    bot.core.robot_reboot_motors('single', 'gripper', True)
    time.sleep(0.5)
    
    # Set operating modes
    bot.core.robot_set_operating_modes('group', 'arm', 'position')
    bot.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
    
    # Set current limit for gripper
    bot.core.robot_set_motor_registers('single', 'gripper', 'current_limit', 300)
    
    # Enable torque
    bot.core.robot_torque_enable('group', 'arm', True)
    bot.core.robot_torque_enable('single', 'gripper', True)
    
    print(f"Gripper setup complete!")
    time.sleep(0.5)
    
def send_gripper_command(bot: InterbotixManipulatorXS, position: float):
    """
    Send a gripper position command.
    
    Args:
        bot: The InterbotixManipulatorXS robot instance
        position: Gripper position in radians
                 For follower: 0.75 (closed) to 1.74 (open)
    """
    msg = JointSingleCommand()
    msg.name = 'gripper'
    msg.cmd = position
    
    bot.gripper.core.pub_single.publish(msg)


def normalize_to_radians(normalized_value: float, 
                        joint_close: float = 0.75, 
                        joint_open: float = 1.74) -> float:
    """
    Convert normalized value (0-1) to radians.
    
    Args:
        normalized_value: Value between 0 (closed) and 1 (open)
        joint_close: Closed position in radians
        joint_open: Open position in radians
        
    Returns:
        Position in radians
    """
    return normalized_value * (joint_open - joint_close) + joint_close


def main():
    parser = argparse.ArgumentParser(
        description='Send JointSingleCommand to robot gripper'
    )
    parser.add_argument(
        '--robot_name',
        type=str,
        default='follower_left',
        help='Robot name (e.g., follower_left, follower_right, follower_solo)'
    )
    parser.add_argument(
        '--robot_model',
        type=str,
        default='vx300s',
        help='Robot model (e.g., wx250s, vx300s)'
    )
    parser.add_argument(
        '--position',
        type=float,
        required=True,
        help='Gripper position (0.75-1.74 rad, or 0-1 if --normalized)'
    )
    parser.add_argument(
        '--normalized',
        action='store_true',
        help='If set, position is normalized (0=closed, 1=open)'
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=2.0,
        help='Duration to publish command (seconds)'
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=30.0,
        help='Publishing rate (Hz)'
    )
    parser.add_argument(
        '--skip_setup',
        action='store_true',
        help='Skip gripper setup (use if already configured)'
    )
    
    args = parser.parse_args()
    
    # Convert normalized to radians if needed
    if args.normalized:
        position_rad = normalize_to_radians(args.position)
        print(f'Converting normalized {args.position} -> {position_rad:.4f} rad')
    else:
        position_rad = args.position
    
    # Validate range
    if position_rad < 0.5 or position_rad > 2.0:
        print(f'Warning: Position {position_rad:.4f} rad is outside typical range [0.75, 1.74]')
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create InterbotixRobotNode
        node = create_interbotix_global_node('gripper_commander')
        
        # Create robot instance
        print(f'Initializing robot {args.robot_name} (model: {args.robot_model})...')
        bot = InterbotixManipulatorXS(
            robot_model=args.robot_model,
            robot_name=args.robot_name,
            node=node,
            iterative_update_fk=False,
        )
        
        # Setup gripper (unless skipped)
        if not args.skip_setup:
            setup_gripper(bot)
        else:
            print("Skipping gripper setup...")
        
        # Send command continuously for specified duration
        start_time = time.time()
        dt = 1.0 / args.rate
        
        print(f'\nSending gripper command:')
        print(f'  Robot: {args.robot_name}')
        print(f'  Position: {position_rad:.4f} rad')
        print(f'  Duration: {args.duration:.1f} seconds')
        print(f'  Rate: {args.rate:.1f} Hz\n')
        
        count = 0
        while time.time() - start_time < args.duration:
            send_gripper_command(bot, position_rad)
            count += 1
            if count % int(args.rate) == 0:  # Print once per second
                print(f'  Sent {count} commands... (position={position_rad:.4f} rad)')
            time.sleep(dt)
        
        print(f'\nCommand sent successfully! Total commands: {count}')
        
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    
    finally:
        # Cleanup
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

