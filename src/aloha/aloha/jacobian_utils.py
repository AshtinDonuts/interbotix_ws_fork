"""
Get the Jacobian from a PRE-RECORDED dataset of joint state observation values.

This is useful for Visualization.

In practice, for training and inference we compute on the fly.
For live computation, we integrate on the ACT codebase side.

It is important to use the observation values as these correspond to the follower arm's values.

conda env :


Usage:
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash
python /home/khw/interbotix_ws/src/aloha/aloha/jacobian_utils.py -j body

"""

import argparse
import os
from pathlib import Path
from typing import List, Dict
import numpy as np
import pandas as pd
import h5py
import modern_robotics as mr
import matplotlib.pyplot as plt
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
from aloha.robot_utils import load_yaml_file

#
# File Organization:
# --------------------------------
# - Data / viz methods
# - Jacobian computation methods
# - main method example for offline computation + Viz


# ================================================ 
# DATA / VIZ UTILS
# 

def load_joint_states_from_parquet(file_path: str) -> np.ndarray:
    """
    Load joint state observations from a parquet file.
    
    :param file_path: Path to the parquet file containing observations
    :return: Array of qpos values (each row is a timestep)
    """
    df = pd.read_parquet(file_path)
    
    # Check if qpos column exists
    if 'qpos' in df.columns:
        # If qpos is stored as arrays/lists, convert to numpy array
        qpos_data = df['qpos'].values
        if isinstance(qpos_data[0], (list, np.ndarray)):
            return np.array([np.array(q) for q in qpos_data])
        else:
            return qpos_data
    elif '/observations/qpos' in df.columns:
        qpos_data = df['/observations/qpos'].values
        if isinstance(qpos_data[0], (list, np.ndarray)):
            return np.array([np.array(q) for q in qpos_data])
        else:
            return qpos_data
    else:
        raise ValueError(f"Could not find 'qpos' or '/observations/qpos' column in parquet file. Available columns: {df.columns.tolist()}")


def load_joint_states_from_hdf5(file_path: str) -> np.ndarray:
    """
    Load joint state observations from an HDF5 file.
    
    :param file_path: Path to the HDF5 file containing observations
    :return: Array of qpos values (num_timesteps, qpos_dim) where each row is a timestep
    :raises FileNotFoundError: If the file does not exist
    :raises ValueError: If qpos data is not found in the file
    """
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"HDF5 file not found: {file_path}")
    
    try:
        with h5py.File(file_path, 'r') as root:
            # Check for qpos in observations group (standard location)
            if '/observations/qpos' in root:
                qpos = root['/observations/qpos'][()]
            elif 'qpos' in root:
                qpos = root['qpos'][()]
            else:
                # List available keys for debugging
                def print_keys(name, obj):
                    if isinstance(obj, h5py.Dataset):
                        return f"  {name} (shape: {obj.shape})"
                    return None
                
                print("Available keys in HDF5 file:")
                available_keys = []
                def collect_keys(name, obj):
                    if isinstance(obj, h5py.Dataset):
                        available_keys.append(f"  {name} (shape: {obj.shape})")
                root.visititems(collect_keys)
                for key in available_keys:
                    print(key)
                raise ValueError(f"Could not find '/observations/qpos' or 'qpos' in HDF5 file: {file_path}")
    except OSError as e:
        raise ValueError(f"Error reading HDF5 file {file_path}: {e}")
    
    # Ensure it's a 2D array (timesteps, joint_dim)
    if qpos.ndim == 1:
        qpos = qpos.reshape(1, -1)
    elif qpos.ndim != 2:
        raise ValueError(f"Expected 1D or 2D qpos array, got {qpos.ndim}D array with shape {qpos.shape}")
    
    return qpos


def save_jacobians(jacobians: np.ndarray, output_path: str):
    """
    Save Jacobians to a file.
    
    :param jacobians: Array of Jacobians (num_timesteps, 6, num_joints)
    :param output_path: Path to save the output file
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Save as numpy file
    np.save(output_path, jacobians)
    print(f"Saved Jacobians to {output_path}")
    
    # Also save as parquet for easier inspection
    parquet_path = output_path.with_suffix('.parquet')
    # Flatten for parquet (each row is a timestep, columns are flattened Jacobian)
    df = pd.DataFrame(jacobians.reshape(jacobians.shape[0], -1))
    df.to_parquet(parquet_path)
    print(f"Saved Jacobians (flattened) to {parquet_path}")


def visualize_jacobians(jacobians: np.ndarray, output_path: str = None, jacobian_type: str = 'space'):
    """
    Visualize Jacobian values over time.
    
    :param jacobians: Array of Jacobians (num_timesteps, 6, num_joints)
    :param output_path: Optional path to save the visualization
    :param jacobian_type: Type of Jacobian ('space' or 'body') for labeling
    """
    num_timesteps, num_rows, num_cols = jacobians.shape
    
    # Create subplots: one for each row of the Jacobian (6 rows)
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()
    
    row_labels = ['vx', 'vy', 'vz', 'wx', 'wy', 'wz']
    
    for row_idx in range(num_rows):
        ax = axes[row_idx]
        
        # Plot each column (joint) as a separate line
        for col_idx in range(num_cols):
            ax.plot(jacobians[:, row_idx, col_idx], label=f'Joint {col_idx+1}', alpha=0.7)
        
        ax.set_xlabel('Timestep')
        ax.set_ylabel('Jacobian Value')
        ax.set_title(f'{row_labels[row_idx]} (Row {row_idx}) - {jacobian_type.capitalize()} Jacobian')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.suptitle(f'{jacobian_type.capitalize()} Jacobian Values Over Time', fontsize=14, y=1.02)
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Saved visualization to {output_path}")
    else:
        plt.show()
    
    # Also create a heatmap visualization
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    
    # Average Jacobian magnitude over time
    avg_jacobian = np.mean(np.abs(jacobians), axis=0)
    im1 = axes[0].imshow(avg_jacobian, aspect='auto', cmap='viridis')
    axes[0].set_xlabel('Joint')
    axes[0].set_ylabel('Jacobian Row (vx,vy,vz,wx,wy,wz)')
    axes[0].set_title(f'Average |{jacobian_type.capitalize()} Jacobian| Magnitude')
    axes[0].set_yticks(range(6))
    axes[0].set_yticklabels(['vx', 'vy', 'vz', 'wx', 'wy', 'wz'])
    plt.colorbar(im1, ax=axes[0])
    
    # Jacobian condition number over time
    condition_numbers = [np.linalg.cond(J) for J in jacobians]
    axes[1].plot(condition_numbers)
    axes[1].set_xlabel('Timestep')
    axes[1].set_ylabel('Condition Number')
    axes[1].set_title(f'{jacobian_type.capitalize()} Jacobian Condition Number Over Time')
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_path:
        heatmap_path = Path(output_path).with_name(Path(output_path).stem + '_heatmap.png')
        plt.savefig(heatmap_path, dpi=150)
        print(f"Saved heatmap visualization to {heatmap_path}")
    else:
        plt.show()


# ===============================
# JACOBIAN COMPUTATOIN UTILS
#

def get_robot_slist(robot_model: str) -> np.ndarray:
    """
    Get the screw axes (Slist) for a robot model from Modern Robotics descriptions.
    
    :param robot_model: Robot model name (e.g., 'aloha_vx300s', 'aloha_wx250s')
    :return: Slist array (6 x num_joints)
    """
    try:
        robot_des = getattr(mrd, robot_model)
        return robot_des.Slist
    except AttributeError:
        raise ValueError(f"Robot model '{robot_model}' not found in mr_descriptions. Available models: {[name for name in dir(mrd) if not name.startswith('_') and hasattr(getattr(mrd, name), 'Slist')]}")


def get_robot_blist(robot_model: str) -> np.ndarray:
    """
    Get the body screw axes (Blist) for a robot model from Modern Robotics descriptions.
    Blist is computed from Slist and M using the Adjoint transformation.
    
    :param robot_model: Robot model name (e.g., 'aloha_vx300s', 'aloha_wx250s')
    :return: Blist array (6 x num_joints)
    """
    try:
        robot_des = getattr(mrd, robot_model)
        slist = robot_des.Slist
        M = robot_des.M
        
        # Convert Slist to Blist using: Blist = Adjoint(TransInv(M)) * Slist
        # TransInv(M) is the inverse of M
        M_inv = mr.TransInv(M)
        Ad_M_inv = mr.Adjoint(M_inv)
        
        # Blist = Adjoint(M_inv) @ Slist
        blist = Ad_M_inv @ slist
        return blist
    except AttributeError:
        raise ValueError(f"Robot model '{robot_model}' not found in mr_descriptions. Available models: {[name for name in dir(mrd) if not name.startswith('_') and hasattr(getattr(mrd, name), 'Slist')]}")


def extract_arm_joints(qpos: np.ndarray, num_arm_joints: int = 6, num_robots: int = 1) -> np.ndarray:
    """
    Extract arm joint positions from qpos array.
    qpos structure: [arm_joints (6), gripper (1)] for each robot
    
    :param qpos: Full qpos array for one timestep
    :param num_arm_joints: Number of arm joints per robot (default: 6)
    :param num_robots: Number of robots in the dataset (default: 1)
    :return: Array of arm joint positions (num_arm_joints,)
    """
    joints_per_robot = num_arm_joints + 1  # +1 for gripper
    
    # For now, extract joints from the first robot only
    # If multiple robots, you may want to compute Jacobian for each
    if len(qpos) >= num_arm_joints:
        # If single robot or first robot's joints
        return qpos[:num_arm_joints]
    else:
        raise ValueError(f"qpos length {len(qpos)} is less than expected {num_arm_joints} arm joints")


def compute_jacobians(qpos_array: np.ndarray, screw_axes: np.ndarray, jacobian_type : str, num_arm_joints: int = 6) -> np.ndarray:
    """
    Compute Jacobians for each timestep.
    
    :param qpos_array: Array of qpos values (num_timesteps, qpos_dim)
    :param screw_axes: Screw axes matrix (Slist for space, Blist for body) (6 x num_joints)
    :param num_arm_joints: Number of arm joints per robot
    :param jacobian_type: Type of Jacobian to compute - 'space' or 'body'
    :return: Array of Jacobians (num_timesteps, 6, num_joints)
    """
    num_timesteps = len(qpos_array)
    num_joints = screw_axes.shape[1]
    
    jacobians = np.zeros((num_timesteps, 6, num_joints))
    
    for i, qpos in enumerate(qpos_array):
        arm_joints = extract_arm_joints(qpos, num_arm_joints)
        
        # Ensure we have the right number of joints
        if len(arm_joints) != num_joints:
            raise ValueError(f"Number of arm joints ({len(arm_joints)}) does not match screw axes columns ({num_joints})")
        
        # Compute appropriate Jacobian type
        if jacobian_type.lower() == 'space':
            J = mr.JacobianSpace(screw_axes, arm_joints)
        elif jacobian_type.lower() == 'body':
            J = mr.JacobianBody(screw_axes, arm_joints)
        else:
            raise ValueError(f"Invalid jacobian_type: {jacobian_type}. Must be 'space' or 'body'")
        
        assert isinstance(J, np.ndarray) and J.shape[0] == 6 and J.shape[1] == num_joints, \
            f"Jacobian shape is {J.shape}, expected (6, {num_joints})"
        jacobians[i] = J
    
    return jacobians


# ================================================


def main(args: Dict):
    """
    Main function to compute Jacobians from recorded joint states.
    
    :param args: Dictionary containing command-line arguments
    """
    # Load file and configurations
    joint_state_file = args.get('input_file', 'placeholder_joint_states.parquet')
    robot_config_name = args.get('robot', 'aloha_solo')
    output_file_arg = args.get('output_file', 'jacobians.npy')
    visualize = args.get('visualize', True)
    
    # Create jacobian_results directory if using default output path
    # Check if output_file is just a filename (default case) or contains a path
    output_path = Path(output_file_arg)
    if output_file_arg == 'jacobians.npy' or (output_path.name == 'jacobians.npy' and output_path.parent == Path('.')):
        # Using default path - create jacobian_results directory
        results_dir = Path('jacobian_results')
        results_dir.mkdir(exist_ok=True)
        output_file = str(results_dir / 'jacobians.npy')
        print(f"Using default output directory: {results_dir}")
    else:
        # Custom output path provided
        output_file = output_file_arg
        # Ensure parent directory exists for custom output path
        output_path.parent.mkdir(parents=True, exist_ok=True)
    
    print(f"Loading joint states from: {joint_state_file}")
    print(f"Using robot configuration: {robot_config_name}")
    print(f"Output will be saved to: {output_file}")
    
    # Load robot configuration to get robot model
    base_path = Path(__file__).resolve().parent.parent / "config"
    config = load_yaml_file("robot", robot_config_name, base_path).get('robot', {})
    
    # Get follower robot model (assuming first follower)
    follower_arms = config.get('follower_arms', [])
    if not follower_arms:
        raise ValueError("No follower arms found in robot configuration")
    
    robot_model = follower_arms[0].get('model', 'aloha_vx300s')
    print(f"Using robot model: {robot_model}")

    # import pdb
    # pdb.set_trace()
    
    # Get jacobian type from arguments (required)
    jacobian_type = args.get('jacobian_type', '').lower()
    if jacobian_type not in ['space', 'body']:
        raise ValueError(f"jacobian_type is required and must be 'space' or 'body', got: {jacobian_type}")
    
    # Get screw axes for the robot model (Slist for space, Blist for body)
    if jacobian_type == 'space':
        screw_axes = get_robot_slist(robot_model)
        print(f"Using space Jacobian (Slist)")
    else:  # body
        screw_axes = get_robot_blist(robot_model)
        print(f"Using body Jacobian (Blist)")
    
    num_arm_joints = screw_axes.shape[1]
    print(f"Robot has {num_arm_joints} arm joints")
    
    # Load joint state observation values
    # Automatically detect file type based on extension
    file_path = Path(joint_state_file)
    file_ext = file_path.suffix.lower()
    
    try:
        if file_ext in ['.hdf5', '.h5']:
            print(f"Loading from HDF5 file: {joint_state_file}")
            qpos_array = load_joint_states_from_hdf5(joint_state_file)
        elif file_ext == '.parquet':
            print(f"Loading from Parquet file: {joint_state_file}")
            qpos_array = load_joint_states_from_parquet(joint_state_file)
        else:
            # Try to auto-detect by attempting to open as HDF5 first, then parquet
            print(f"Unknown file extension '{file_ext}', attempting to auto-detect format...")
            try:
                qpos_array = load_joint_states_from_hdf5(joint_state_file)
                print("Successfully loaded as HDF5 file")
            except (OSError, KeyError, ValueError):
                try:
                    qpos_array = load_joint_states_from_parquet(joint_state_file)
                    print("Successfully loaded as Parquet file")
                except Exception as e:
                    raise ValueError(f"Could not load file as HDF5 or Parquet. Error: {e}")
        
        print(f"Loaded {len(qpos_array)} timesteps")
        if len(qpos_array) > 0:
            if isinstance(qpos_array, np.ndarray):
                print(f"qpos shape: {qpos_array.shape}")
            else:
                print(f"qpos shape: {qpos_array[0].shape if len(qpos_array) > 0 else 'N/A'}")
    except FileNotFoundError:
        print(f"Warning: File {joint_state_file} not found. Using placeholder data.")
        # Create placeholder data for testing
        num_timesteps = 100
        num_joints_per_robot = num_arm_joints + 1  # +1 for gripper
        qpos_array = np.random.randn(num_timesteps, num_joints_per_robot) * 0.1
        print(f"Generated {num_timesteps} placeholder timesteps")
    
    # Compute the Jacobian at each corresponding timeframe
    print(f"Computing {jacobian_type} Jacobians...")
    jacobians = compute_jacobians(qpos_array, screw_axes, jacobian_type, num_arm_joints)
    print(f"Computed {len(jacobians)} {jacobian_type} Jacobians, shape: {jacobians.shape}")
    
    # Print first Jacobian as example
    print(f'\nFirst Jacobian (timestep 0):\n{jacobians[0]}')
    
    # Save to output file path
    save_jacobians(jacobians, output_file)
    
    # Visualize the Jacobian values
    if visualize:
        output_path = Path(output_file)
        viz_path = output_path.with_suffix('.png')
        visualize_jacobians(jacobians, str(viz_path), jacobian_type)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compute Jacobians from recorded joint states')
    parser.add_argument(
        '-i', '--input_file',
        type=str,
        default='placeholder_joint_states.parquet',
        help='Path to input file containing joint state observations (supports .parquet, .hdf5, .h5)'
    )
    parser.add_argument(
        '-r', '--robot',
        type=str,
        default='aloha_solo',
        help='Robot configuration name (e.g., aloha_solo, aloha_stationary, aloha_mobile)'
    )
    parser.add_argument(
        '-o', '--output_file',
        type=str,
        default='jacobians.npy',
        help='Path to save output Jacobians file (default: jacobian_results/jacobians.npy)'
    )
    parser.add_argument(
        '--no-visualize',
        action='store_true',
        help='Disable visualization'
    )
    parser.add_argument(
        '-j', '--jacobian_type',
        type=str,
        required=True,
        choices=['space', 'body'],
        help='Type of Jacobian to compute: "space" or "body" (required)'
    )
    
    print(f'Initiating script: {__file__}')
    args_dict = vars(parser.parse_args())
    args_dict['visualize'] = not args_dict.pop('no_visualize', False)
    main(args_dict)
