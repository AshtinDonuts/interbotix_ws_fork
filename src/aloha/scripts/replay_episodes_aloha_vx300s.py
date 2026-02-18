#!/usr/bin/env python3
"""
Replay saved robot episodes using direct joint position control.

This script loads recorded actions from an HDF5 dataset and replays them on the robot
using direct joint position control. The robot is automatically positioned to the
initial state from the recorded trajectory before replay begins.

This version is designed to work with a robot already running via:
    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=aloha_vx300s

Example usage:
    # Replay episode 0 from a dataset directory:
    python replay_episodes_aloha_vx300s.py --dataset_dir /mnt/c2d9b23a-b03e-4fdb-82ad-59f039ec9e3e/khw/puzzle_ssil/ --episode_idx 1

    # Replay episode 5:
    python replay_episodes_aloha_vx300s.py --dataset_dir /path/to/dataset --episode_idx 5

    # Replay episode 0 (default):
    python replay_episodes_aloha_vx300s.py --dataset_dir /path/to/dataset
"""

import argparse
import os
import time
from typing import Dict

import h5py
from aloha.robot_utils import (
    move_grippers,
    move_arms,
    JOINT_NAMES,
    FOLLOWER_GRIPPER_JOINT_OPEN,
    FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN,
)
from interbotix_common_modules.common_robot.robot import (
    get_interbotix_global_node,
    create_interbotix_global_node,
    robot_shutdown,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from pathlib import Path


# Define joint and gripper state names for tracking purposes
STATE_NAMES = JOINT_NAMES + ['gripper', 'left_finger', 'right_finger']


def main(args: Dict[str, any]) -> None:
    """
    Main function to replay a saved episode for the robot.
    Loads actions from an HDF5 file and applies them to a robot already running via launch file.

    :param args: Dictionary of command-line arguments, including:
        - 'dataset_dir' (str): Path to the directory containing episode datasets.
        - 'episode_idx' (int): Index of the episode file to load.
        - 'robot_name' (str): Name of the robot (defaults to 'aloha_vx300s').
        - 'robot_model' (str): Model of the robot (defaults to 'aloha_vx300s').
        - 'fps' (float): Control frequency in Hz (defaults to 50).
    """
    # Get robot configuration from arguments
    robot_name = args.get('robot_name', 'aloha_vx300s')
    robot_model = args.get('robot_model', 'aloha_vx300s')
    fps = args.get('fps', 50.0)
    
    # Set the timestep duration for the environment update frequency
    dt = 1.0 / fps

    # Construct dataset path
    dataset_dir = args['dataset_dir']
    episode_idx = args['episode_idx']
    dataset_name = f'episode_{episode_idx}'
    dataset_path = os.path.join(dataset_dir, dataset_name + '.hdf5')

    # Check if dataset exists, and exit if not
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    # Load actions from the dataset
    with h5py.File(dataset_path, 'r') as root:
        actions = root['/action'][()]
        # Check if base_actions exist (for mobile robots)
        base_actions = None
        if '/base_action' in root:
            base_actions = root['/base_action'][()]

    # Extract initial positions from the first action in the dataset
    # This ensures we start from the exact same position where the trajectory was recorded
    if len(actions) == 0:
        print('Error: Dataset contains no actions')
        exit()
    
    initial_action = actions[0]
    
    # Get or create the ROS node (connect to existing if launch file is running)
    try:
        node = get_interbotix_global_node()
        if node is None:
            raise AttributeError
        print('Connected to existing ROS node from launch file')
    except (NameError, AttributeError, RuntimeError):
        # If no global node exists, create one
        node = create_interbotix_global_node('aloha_replay')
        print('Created new ROS node (launch file may not be running)')

    # Connect to the robot that's already running from the launch file
    print(f'Connecting to robot: {robot_name} (model: {robot_model})')
    robot = InterbotixManipulatorXS(
        robot_model=robot_model,
        robot_name=robot_name,
        node=node,
        iterative_update_fk=False,
    )

    # Configure robot for position control
    robot.core.robot_reboot_motors('single', 'gripper', True)
    robot.core.robot_set_operating_modes('group', 'arm', 'position')
    robot.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
    robot.core.robot_torque_enable('group', 'arm', True)
    robot.core.robot_torque_enable('single', 'gripper', True)
    
    # Determine action format - check if it's multi-robot or single robot
    # Single robot action: 7 values (6 joints + 1 gripper)
    # Multi-robot action: 7 * num_robots values
    action_dim = len(initial_action)
    state_len = 7  # 6 joints + 1 gripper
    
    if action_dim == state_len:
        # Single robot format
        num_robots = 1
        print('Detected single robot action format')
    elif action_dim % state_len == 0:
        # Multi-robot format - use first robot's actions
        num_robots = action_dim // state_len
        print(f'Detected multi-robot action format ({num_robots} robots), using first robot')
    else:
        print(f'Error: Unexpected action dimension {action_dim}, expected multiple of {state_len}')
        exit()
    
    # Extract initial joint and gripper positions
    if num_robots == 1:
        bot_action = initial_action
    else:
        # Use first robot's action
        bot_action = initial_action[:state_len]
    
    # First 6 values are arm joint positions, last value is normalized gripper
    initial_joint_positions = bot_action[:-1]
    # Unnormalize gripper position
    gripper_normalized = bot_action[-1]
    gripper_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(gripper_normalized)
    
    # Move robot to initial position from the dataset (safely, with smooth trajectory)
    print('Moving robot to initial position from dataset...')
    move_arms(
        bot_list=[robot],
        target_pose_list=[initial_joint_positions],
        dt=dt,
        moving_time=3.0,  # Smooth 3-second movement to initial position
    )
    
    # Set initial gripper position
    move_grippers(
        [robot],
        [gripper_joint],
        moving_time=1.0,
        dt=dt,
    )
    
    print('Robot positioned at initial trajectory state. Starting replay...')
    time0 = time.time()

    # Create gripper command message
    gripper_command = JointSingleCommand(name='gripper')
    
    # Execute each action in the episode
    for action in actions:
        time1 = time.time()
        
        # Extract action for this robot
        if num_robots == 1:
            bot_action = action
        else:
            # Use first robot's action
            bot_action = action[:state_len]
        
        # Extract joint positions and gripper
        joint_positions = bot_action[:-1]
        gripper_normalized = bot_action[-1]
        gripper_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(gripper_normalized)
        
        # Move arm to joint positions (non-blocking for real-time control)
        robot.arm.set_joint_positions(joint_positions, moving_time=dt, accel_time=dt*0.3, blocking=False)
        
        # Move gripper
        gripper_command.cmd = gripper_joint
        robot.gripper.core.pub_single.publish(gripper_command)
        
        # Sleep to maintain control frequency
        time.sleep(max(0, dt - (time.time() - time1)))

    # Print average frames per second
    print(f'Avg fps: {len(actions) / (time.time() - time0)}')

    # Move gripper to open position after replay
    move_grippers([robot], [FOLLOWER_GRIPPER_JOINT_OPEN], moving_time=0.5, dt=dt)
    
    # Note: We don't call robot_shutdown() here since the launch file is managing the robot
    print('Replay complete!')


if __name__ == '__main__':
    # Define command-line arguments
    parser = argparse.ArgumentParser(
        description="Replays a saved episode for the robot using direct joint position control. "
                    "Designed to work with a robot already running via launch file.",
        epilog="""
Examples:
  %(prog)s --dataset_dir /path/to/dataset --episode_idx 0
  %(prog)s --dataset_dir /path/to/dataset --episode_idx 5
  %(prog)s --dataset_dir /path/to/dataset --robot_name aloha_vx300s --robot_model aloha_vx300s
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    parser.add_argument(
        '--dataset_dir',
        action='store',
        type=str,
        help='Path to the directory containing the dataset.',
        required=True,
    )
    parser.add_argument(
        '--episode_idx',
        action='store',
        type=int,
        help='Index of the episode to replay.',
        default=0,
        required=False,
    )
    parser.add_argument(
        '--robot_name',
        action='store',
        type=str,
        help='Name of the robot (must match launch file robot_name, defaults to aloha_vx300s).',
        default='aloha_vx300s',
        required=False,
    )
    parser.add_argument(
        '--robot_model',
        action='store',
        type=str,
        help='Model of the robot (defaults to aloha_vx300s).',
        default='aloha_vx300s',
        required=False,
    )
    parser.add_argument(
        '--fps',
        action='store',
        type=float,
        help='Control frequency in Hz (defaults to 50.0).',
        default=50.0,
        required=False,
    )

    # Execute main function with parsed arguments
    main(vars(parser.parse_args()))
