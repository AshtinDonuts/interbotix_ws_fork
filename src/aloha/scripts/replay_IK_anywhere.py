#!/usr/bin/env python3
"""
Replay saved robot episodes using end-effector pose control (IK) from current robot pose.

This script loads recorded actions from an HDF5 dataset and replays them on the robot
using inverse kinematics to control end-effector poses. Instead of moving to the initial
position from the dataset, it starts from the current robot pose and recenters the trajectory
by computing the offset between the first position in the trajectory and the current pose.

The trajectory is de-meaned (recentered) using the first position as the origin, then
replayed relative to the current robot pose.

Example usage:
    python3 replay_IK_anywhere.py --robot aloha_solo --dataset_dir /mnt/c2d9b23a-b03e-4fdb-82ad-59f039ec9e3e/khw/puzzle_ssil/ --episode_idx 1
"""

import argparse
import os
import time
from typing import Dict, List

import h5py
import numpy as np
from aloha.real_env import make_real_env
from aloha.robot_utils import (
    move_grippers,
    load_yaml_file,
    FOLLOWER_GRIPPER_JOINT_OPEN,
    FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN,
)
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
import modern_robotics as mr
from pathlib import Path


def compute_ee_pose_from_joints(robot: InterbotixManipulatorXS, joint_positions: np.ndarray) -> np.ndarray:
    """
    Compute end-effector pose from joint positions using forward kinematics.

    :param robot: The robot manipulator instance.
    :param joint_positions: Array of joint positions in radians.
    :return: 4x4 transformation matrix representing the end-effector pose.
    """
    # Convert to list format expected by FKinSpace
    joint_list = joint_positions.tolist() if isinstance(joint_positions, np.ndarray) else joint_positions
    return mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joint_list)


def compute_recentered_poses(
    original_poses: List[np.ndarray],
    current_pose: np.ndarray,
    first_pose: np.ndarray
) -> List[np.ndarray]:
    """
    Recenter poses using the first pose as origin, then apply offset to current pose.
    Both position and orientation are recentered relative to the current pose.
    
    :param original_poses: List of 4x4 transformation matrices from the trajectory.
    :param current_pose: 4x4 transformation matrix of the current robot pose.
    :param first_pose: 4x4 transformation matrix of the first pose in the trajectory.
    :return: List of recentered 4x4 transformation matrices.
    """
    # For each pose, recenter both position and orientation relative to current pose
    first_rotation = first_pose[:3, :3]
    current_rotation = current_pose[:3, :3]
    
    recentered_poses = []
    for pose in original_poses:
        # Compute relative position from first pose
        relative_position = pose[:3, 3] - first_pose[:3, 3]
        
        # Compute relative rotation from first pose to this pose
        # R_pose = R_relative * R_first, so R_relative = R_pose * R_first^T
        pose_rotation = pose[:3, :3]
        relative_rotation = pose_rotation @ first_rotation.T
        
        # Apply relative rotation to current orientation
        # R_new = R_relative * R_current (apply the same relative rotation to current as was from first to pose)
        new_rotation = relative_rotation @ current_rotation
        
        # Construct new pose
        new_pose = np.eye(4)
        new_pose[:3, 3] = current_pose[:3, 3] + relative_position
        new_pose[:3, :3] = new_rotation
        
        recentered_poses.append(new_pose)
    
    return recentered_poses


def main(args: Dict[str, any]) -> None:
    """
    Main function to replay a saved episode starting from the current robot pose.
    The trajectory is recentered using the first position as origin, then replayed
    relative to the current robot pose.

    :param args: Dictionary of command-line arguments, including:
        - 'dataset_dir' (str): Path to the directory containing episode datasets.
        - 'episode_idx' (int): Index of the episode file to load.
        - 'robot' (str): Robot configuration name (e.g., 'aloha_solo', 'aloha_stationary', 'aloha_mobile').
    """
    # Load robot configuration
    robot_base = args.get('robot', '')

    base_path = Path(__file__).resolve().parent.parent / "config"

    config = load_yaml_file('robot', robot_base, base_path).get('robot', {})
    is_mobile = config.get('base', False)

    # Set the timestep duration for the environment update frequency
    dt = 1 / config.get('fps', 50)

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
        base_actions = root['/base_action'][()] if is_mobile else None

    if len(actions) == 0:
        print('Error: Dataset contains no actions')
        exit()

    # Initialize the ROS node and create the real environment
    node = create_interbotix_global_node('aloha')
    env = make_real_env(node, setup_robots=False,
                        setup_base=is_mobile, config=config)

    # Set mobile base motor torque if applicable
    if is_mobile:
        env.base.base.set_motor_torque(True)
    robot_startup(node)

    # Configure and initialize follower robots
    follower_robots = {}
    follower_bots_list = []  # List version for move_arms
    for name, bot in env.robots.items():
        if 'follower' in name:
            bot.core.robot_reboot_motors('single', 'gripper', True)
            # Use position mode for end-effector pose control
            bot.core.robot_set_operating_modes('group', 'arm', 'position')
            bot.core.robot_set_operating_modes(
                'single', 'gripper', 'current_based_position')
            bot.core.robot_torque_enable('group', 'arm', True)
            bot.core.robot_torque_enable('single', 'gripper', True)
            follower_robots[name] = bot
            follower_bots_list.append(bot)

    # Reset the environment (fake=True skips joint/gripper movement)
    env.reset(fake=True)
    
    # Get follower robot names and calculate state length per robot
    follower_names = list(follower_robots.keys())
    num_followers = len(follower_names)
    if num_followers == 0:
        print('Error: No follower robots found')
        exit()
    
    state_len = int(len(actions[0]) / num_followers)
    
    # Get current robot poses (starting positions)
    print('Getting current robot poses...')
    current_poses = []
    for name in follower_names:
        bot = follower_robots[name]
        current_pose = bot.arm.get_ee_pose()
        current_poses.append(current_pose)
        print(f'  {name} current pose position: {current_pose[:3, 3]}')
    
    # Compute all end-effector poses from the trajectory
    print('Computing end-effector poses from trajectory...')
    trajectory_poses = [[] for _ in range(num_followers)]
    first_poses = []
    
    for action in actions:
        index = 0
        for i, name in enumerate(follower_names):
            bot = follower_robots[name]
            bot_action = action[index:index + state_len]
            
            # Extract arm joint positions
            arm_joint_positions = bot_action[:-1]
            
            # Convert joint positions to end-effector pose
            ee_pose = compute_ee_pose_from_joints(bot, arm_joint_positions)
            trajectory_poses[i].append(ee_pose)
            
            index += state_len
    
    # Get first poses for each robot
    for i in range(num_followers):
        first_poses.append(trajectory_poses[i][0])
        print(f'  {follower_names[i]} first pose position: {first_poses[i][:3, 3]}')
    
    # Recenter the trajectory using first pose as origin, then apply to current pose
    print('Recentering trajectory...')
    recentered_poses = []
    for i in range(num_followers):
        recentered = compute_recentered_poses(
            trajectory_poses[i],
            current_poses[i],
            first_poses[i]
        )
        recentered_poses.append(recentered)
        print(f'  {follower_names[i]} recentered first pose position: {recentered[0][:3, 3]}')
    
    print('Starting replay from current pose...')
    time0 = time.time()

    # Create gripper command message
    gripper_command = JointSingleCommand(name='gripper')

    # Execute each action in the episode using recentered end-effector pose control
    if is_mobile:
        for step_idx, (action, base_action) in enumerate(zip(actions, base_actions)):
            time1 = time.time()
            
            # Process each follower robot
            for i, name in enumerate(follower_names):
                bot = follower_robots[name]
                bot_action = action[i * state_len:(i + 1) * state_len]
                
                # Get recentered end-effector pose
                ee_pose = recentered_poses[i][step_idx]
                
                # Extract gripper position
                gripper_normalized = bot_action[-1]
                
                # Set end-effector pose using IK
                _, success = bot.arm.set_ee_pose_matrix(
                    T_sd=ee_pose,
                    custom_guess=bot.arm.get_joint_commands(),
                    execute=True,
                    moving_time=dt,
                    accel_time=dt * 0.5,
                    blocking=False
                )
                
                # Set gripper position
                gripper_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(gripper_normalized)
                gripper_command.cmd = gripper_joint
                bot.gripper.core.pub_single.publish(gripper_command)
            
            # Handle base action if mobile
            if base_action is not None:
                base_action_linear, base_action_angular = base_action
                env.base.base.command_velocity_xyaw(
                    x=base_action_linear, yaw=base_action_angular)
            
            time.sleep(max(0, dt - (time.time() - time1)))
    else:
        for step_idx, action in enumerate(actions):
            time1 = time.time()
            
            # Process each follower robot
            for i, name in enumerate(follower_names):
                bot = follower_robots[name]
                bot_action = action[i * state_len:(i + 1) * state_len]
                
                # Get recentered end-effector pose
                ee_pose = recentered_poses[i][step_idx]
                
                # Extract gripper position
                gripper_normalized = bot_action[-1]
                
                # Set end-effector pose using IK
                _, success = bot.arm.set_ee_pose_matrix(
                    T_sd=ee_pose,
                    custom_guess=bot.arm.get_joint_commands(),
                    execute=True,
                    moving_time=dt,
                    accel_time=dt * 0.5,
                    blocking=False
                )
                
                # Set gripper position
                gripper_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(gripper_normalized)
                gripper_command.cmd = gripper_joint
                bot.gripper.core.pub_single.publish(gripper_command)
            
            time.sleep(max(0, dt - (time.time() - time1)))

    # Print average frames per second
    total_time = time.time() - time0
    avg_fps = len(actions) / total_time if total_time > 0 else 0
    print(f'\nReplay complete!')
    print(f'Total time: {total_time:.2f}s')
    print(f'Average fps: {avg_fps:.2f}')

    # Move follower grippers to open position after replay
    gripper_positions = [FOLLOWER_GRIPPER_JOINT_OPEN] * len(follower_bots_list)

    # Move follower grippers to open position
    move_grippers(follower_bots_list, gripper_positions, moving_time=0.5, dt=dt)
    robot_shutdown(node)


if __name__ == '__main__':
    # Define command-line arguments
    parser = argparse.ArgumentParser(
        description="Replays a saved episode starting from the current robot pose using de-meaned position control.",
        epilog="""
Examples:
  %(prog)s --dataset_dir /path/to/dataset --episode_idx 0 -r aloha_stationary
  %(prog)s --dataset_dir /path/to/dataset --episode_idx 5 -r aloha_mobile
  %(prog)s --dataset_dir /path/to/dataset -r aloha_solo
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
        '-r', '--robot',
        action='store',
        type=str,
        help='Robot configuration name (e.g., aloha_solo, aloha_stationary, aloha_mobile).',
        required=True,
    )

    # Execute main function with parsed arguments
    main(vars(parser.parse_args()))
