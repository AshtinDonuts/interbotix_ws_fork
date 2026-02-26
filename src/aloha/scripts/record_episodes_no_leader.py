#!/usr/bin/env python3

"""
Capture one episode of follower-only data with no leader-follower teleoperation.

The follower arm(s) are placed in a start pose, then their torques are disabled so
that you can move them around directly. While you move the follower(s), this script
records the usual observation streams from ROS (qpos, qvel, effort, optional
gravity/dynamics torques, and images) for use in downstream pipelines.

The dataset format is intentionally kept similar to `record_episodes.py` so that
consumers can reuse most of the same tooling.
"""

import argparse
from pathlib import Path
from typing import Dict, List

import cv2
import h5py
import numpy as np
import os
import subprocess
import time
from tqdm import tqdm

from aloha.real_env import make_real_env
from aloha.robot_utils import (
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    FOLLOWER_GRIPPER_JOINT_OPEN,
    START_ARM_POSE,
    disable_gravity_compensation,
    enable_gravity_compensation,
    load_yaml_file,
    move_arms,
    move_grippers,
    torque_off,
    torque_on,
)
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)


def set_realsense_exposure(camera_name: str, exposure: int = 20000) -> None:
    """
    Set fixed exposure for a RealSense camera and disable auto exposure.

    This mirrors the behavior in `record_episodes.py` so that datasets are
    captured under comparable imaging conditions.
    """
    node_name = f"/{camera_name}/camera"
    try:
        # Disable auto exposure
        subprocess.run(
            ["ros2", "param", "set", node_name, "depth_module.enable_auto_exposure", "false"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        # Set exposure
        subprocess.run(
            ["ros2", "param", "set", node_name, "depth_module.exposure", str(exposure)],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print(f"Set exposure for {camera_name} to {exposure}")
    except Exception as e:
        print(f"Failed to set exposure for {camera_name}: {e}")


def check_episode_index(dataset_dir: str, episode_idx: int, data_suffix: str = "hdf5") -> bool:
    """
    Check if an episode file already exists and ask the user whether to overwrite it.
    """
    episode_file = os.path.join(dataset_dir, f"episode_{episode_idx}.{data_suffix}")
    if os.path.isfile(episode_file):
        user_input = input(
            f"Episode file '{episode_file}' already exists. Do you want to overwrite it? (y/n): "
        ).strip().lower()
        if user_input == "y":
            print(f"Overwriting episode {episode_idx}.")
            return True
        print("Not overwriting the file. Operation aborted.")
        return False
    return True


def get_auto_index(dataset_dir: str, dataset_name_prefix: str = "", data_suffix: str = "hdf5") -> int:
    """
    Find the next available episode index in a dataset directory.
    """
    max_idx = 10000

    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)

    for i in range(max_idx + 1):
        episode_file = os.path.join(
            dataset_dir, f"{dataset_name_prefix}episode_{i}.{data_suffix}"
        )
        if not os.path.isfile(episode_file):
            return i

    raise Exception(f"Error getting auto index, or more than {max_idx} episodes.")


def print_dt_diagnosis(actual_dt_history: List[List[float]]) -> float:
    """
    Diagnose timing statistics for each step in the episode.

    This is kept compatible with the helper in `record_episodes.py` so that
    frequency checks behave similarly.
    """
    arr = np.array(actual_dt_history, dtype=float)
    if arr.shape[0] == 0:
        print("No timing history recorded.")
        return 0.0

    get_action_time = arr[:, 1] - arr[:, 0]
    step_env_time = arr[:, 2] - arr[:, 1]
    total_time = arr[:, 2] - arr[:, 0]

    dt_mean_float = float(np.mean(total_time))
    freq_mean = 1.0 / dt_mean_float if dt_mean_float > 0.0 else 0.0

    print(
        f"Avg freq: {freq_mean:.2f} "
        f"Get obs: {np.mean(get_action_time):.3f} "
        f"Sleep: {np.mean(step_env_time):.3f}"
    )

    return freq_mean


def capture_one_episode_no_leader(
    max_timesteps: int,
    dataset_dir: str,
    dataset_name: str,
    overwrite: bool,
    torque_base: bool = False,
    use_follower_gravity_compensation: bool = False,
    config: Dict | None = None,
    get_gravity_torque: bool = False,
    get_dynamics_torque: bool = False,
    exposure: int | None = None,
) -> bool:
    """
    Capture one follower-only episode.

    The follower arm(s) are first moved to a standard start pose. After you
    confirm via the terminal, either:

    - Torque is disabled on the follower arm(s) so you can physically move them
      with no active control (default behavior), or
    - Follower gravity compensation is enabled via the standard Interbotix
      gravity compensation interface so that the arms feel weightless while you
      move them (`use_follower_gravity_compensation=True`).

    While you move the follower arm(s), this function records:

    - /observations/qpos, /qvel, /effort
    - Optional /observations/gravity_torque
    - Optional /observations/dynamics_torque/*
    - /observations/images/<camera_name> for all configured cameras
    - /action (zeros, shaped like qpos, for compatibility)
    """
    if config is None:
        raise ValueError("Config dictionary must be provided.")

    is_mobile = config.get("base", False)
    dt = 1.0 / config.get("fps", 50)

    # Initialize ROS node and environment
    node = create_interbotix_global_node("aloha")
    env = make_real_env(
        node=node,
        setup_robots=False,
        setup_base=is_mobile,
        torque_base=torque_base,
        config=config,
        bool_gravity_torque=get_gravity_torque,
        bool_dynamics_torque=get_dynamics_torque,
    )
    robot_startup(node)

    # Optionally set camera exposure
    if exposure is not None:
        camera_names_cfg = [
            camera["name"]
            for camera in config.get("cameras", {}).get("camera_instances", [])
        ]
        for cam_name in camera_names_cfg:
            set_realsense_exposure(cam_name, exposure)

    # Prepare dataset path
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, dataset_name)
    if os.path.isfile(dataset_path) and not overwrite:
        print(f"Dataset already exists at {dataset_path}\nHint: Set overwrite to True.")
        robot_shutdown()
        return False

    # Identify follower robots and move them to a start pose
    follower_bots = {
        name: bot for name, bot in env.robots.items() if "follower" in name
    }
    if not follower_bots:
        raise RuntimeError("No follower robots found in env.robots; check robot config.")

    print("Moving follower arm(s) to start pose...")
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms(
        bot_list=list(follower_bots.values()),
        target_pose_list=[start_arm_qpos] * len(follower_bots),
        moving_time=4.0,
        dt=dt,
    )
    move_grippers(
        list(follower_bots.values()),
        [FOLLOWER_GRIPPER_JOINT_CLOSE] * len(follower_bots),
        moving_time=0.5,
        dt=dt,
    )

    if not use_follower_gravity_compensation:
        # Immediately disable gripper torque so the grippers are backdrivable,
        # while keeping the arm joints torqued on until the user confirms start.
        for bot in follower_bots.values():
            bot.core.robot_torque_enable("single", "gripper", False)

        print("\nFollower-only recording.")
        print("The follower arm(s) are now in the start pose.")
        input(
            "Press ENTER to disable torque on the follower arm(s) and begin recording.\n"
            "After that, physically move the follower arm(s); data will be recorded for "
            f"{max_timesteps} timesteps.\n"
        )

        # Disable torque on followers so they can be moved freely
        for bot in follower_bots.values():
            torque_off(bot)
        print("Follower torques disabled. Begin moving the follower arm(s).")
    else:
        print("\nFollower-only recording with follower gravity compensation.")
        print("The follower arm(s) are now in the start pose.")
        input(
            "Press ENTER to ENABLE gravity compensation on the follower arm(s) "
            "and begin recording.\n"
            "After that, physically move the follower arm(s); data will be recorded for "
            f"{max_timesteps} timesteps.\n"
        )

        print("Enabling follower gravity compensation on:")
        for name, bot in follower_bots.items():
            print(f"  - {name}")
            # Ensure torques are enabled before enabling gravity compensation.
            torque_on(bot)
            enable_gravity_compensation(bot)
        print("Follower gravity compensation enabled. Begin moving the follower arm(s).")

    # Derive sizes from the first observation
    initial_obs = env.get_observation()
    total_size = len(initial_obs["qpos"])

    # Initialize storage
    data_dict: Dict[str, List] = {
        "/observations/qpos": [],
        "/observations/qvel": [],
        "/observations/effort": [],
        "/action": [],
    }
    if get_gravity_torque:
        data_dict["/observations/gravity_torque"] = []
    if get_dynamics_torque:
        data_dict["/observations/dynamics_torque/gravity_torques"] = []
        data_dict["/observations/dynamics_torque/kinetic_friction_torques"] = []
        data_dict["/observations/dynamics_torque/static_friction_torques"] = []
        data_dict["/observations/dynamics_torque/dither_speeds"] = []
        data_dict["/observations/dynamics_torque/no_load_currents"] = []

    camera_names = [
        camera["name"] for camera in config.get("cameras", {}).get("camera_instances", [])
    ]
    if camera_names:
        for cam_name in camera_names:
            data_dict[f"/observations/images/{cam_name}"] = []

    actual_dt_history: List[List[float]] = []
    start_time = time.time()

    # Main capture loop
    for _ in tqdm(range(max_timesteps)):
        t0 = time.time()
        obs = env.get_observation()
        t1 = time.time()

        # Core state observations
        data_dict["/observations/qpos"].append(obs["qpos"])
        data_dict["/observations/qvel"].append(obs["qvel"])
        data_dict["/observations/effort"].append(obs["effort"])

        if get_gravity_torque:
            data_dict["/observations/gravity_torque"].append(obs["gravity_torque"])

        if get_dynamics_torque:
            dynamics = obs["dynamics_torque"]
            data_dict["/observations/dynamics_torque/gravity_torques"].append(
                dynamics["gravity_torques"]
            )
            data_dict["/observations/dynamics_torque/kinetic_friction_torques"].append(
                dynamics["kinetic_friction_torques"]
            )
            data_dict["/observations/dynamics_torque/static_friction_torques"].append(
                dynamics["static_friction_torques"]
            )
            data_dict["/observations/dynamics_torque/dither_speeds"].append(
                dynamics["dither_speeds"]
            )
            data_dict["/observations/dynamics_torque/no_load_currents"].append(
                dynamics["no_load_currents"]
            )

        # Images
        if camera_names:
            for cam_name in camera_names:
                image = obs["images"][cam_name]
                if image is None:
                    raise RuntimeError(
                        f"Missing image frame for camera '{cam_name}' at timestep. "
                        "Cannot save dataset with missing frames."
                    )

                # Center crop/resize to 640x480 as in `record_episodes.py`
                h, w = image.shape[:2]
                new_w, new_h = 640, 480

                left = max((w - new_w) // 2, 0)
                top = max((h - new_h) // 2, 0)
                right = left + new_w
                bottom = top + new_h

                cropped = image[top:bottom, left:right]
                image = cropped
                data_dict[f"/observations/images/{cam_name}"].append(image)

        # For compatibility, store a zero "action" vector shaped like qpos
        action = np.zeros(total_size, dtype=np.float32)
        data_dict["/action"].append(action)

        # Sleep to maintain approximate frequency
        before_sleep = time.time()
        remaining = dt - (before_sleep - t0)
        if remaining > 0:
            time.sleep(remaining)
        t2 = time.time()
        actual_dt_history.append([t0, t1, t2])

    # Turn off gravity compensation (if enabled) and ensure follower torques are on
    if use_follower_gravity_compensation:
        print("Disabling follower gravity compensation on follower arm(s)...")
        for name, bot in follower_bots.items():
            try:
                disable_gravity_compensation(bot)
            except Exception as exc:  # noqa: BLE001
                print(
                    f"  Warning: failed to disable gravity compensation on {name}: {exc}"
                )

    for bot in follower_bots.values():
        torque_on(bot)
    print("Follower torques re-enabled on follower arm(s).")

    print(f"Avg fps (wall clock): {max_timesteps / (time.time() - start_time)}")

    # Frequency check
    freq_mean = print_dt_diagnosis(actual_dt_history)
    if freq_mean < 30:
        print(
            f"\n\nfreq_mean is {freq_mean}, lower than 30, re-collecting...\n\n\n\n"
        )
        robot_shutdown()
        return False

    # Prepare to write HDF5 dataset
    camera_names = [
        camera["name"] for camera in config.get("cameras", {}).get("camera_instances", [])
    ]
    COMPRESS = False  # Keep uncompressed like the default in record_episodes.py

    image_shape = None
    if camera_names and not COMPRESS:
        first_cam_name = camera_names[0]
        first_image = data_dict[f"/observations/images/{first_cam_name}"][0]
        if first_image is not None:
            image_shape = first_image.shape

    t0 = time.time()
    with h5py.File(dataset_path + ".hdf5", "w", rdcc_nbytes=1024**2 * 2) as root:
        root.attrs["sim"] = False
        root.attrs["compress"] = COMPRESS
        obs_group = root.create_group("observations")

        # Image datasets
        if camera_names:
            image_group = obs_group.create_group("images")
            for cam_name in camera_names:
                if COMPRESS:
                    # Currently not used, but kept for parity.
                    raise NotImplementedError("Compressed image saving not enabled.")
                else:
                    if image_shape is not None:
                        shape = (max_timesteps, image_shape[0], image_shape[1], image_shape[2])
                        chunks = (1, image_shape[0], image_shape[1], image_shape[2])
                    else:
                        shape = (max_timesteps, 480, 640, 3)
                        chunks = (1, 480, 640, 3)
                _ = image_group.create_dataset(
                    cam_name, shape, dtype="uint8", chunks=chunks
                )

        # Core state/action datasets
        _ = obs_group.create_dataset("qpos", (max_timesteps, total_size))
        _ = obs_group.create_dataset("qvel", (max_timesteps, total_size))
        _ = obs_group.create_dataset("effort", (max_timesteps, total_size))
        _ = root.create_dataset("action", (max_timesteps, total_size))

        if get_gravity_torque:
            _ = obs_group.create_dataset("gravity_torque", (max_timesteps, total_size))

        if get_dynamics_torque:
            dynamics_group = obs_group.create_group("dynamics_torque")
            _ = dynamics_group.create_dataset("gravity_torques", (max_timesteps, total_size))
            _ = dynamics_group.create_dataset(
                "kinetic_friction_torques", (max_timesteps, total_size)
            )
            _ = dynamics_group.create_dataset(
                "static_friction_torques", (max_timesteps, total_size)
            )
            _ = dynamics_group.create_dataset("dither_speeds", (max_timesteps, total_size))
            _ = dynamics_group.create_dataset(
                "no_load_currents", (max_timesteps, total_size)
            )

        # Write all arrays from data_dict
        for name, array in data_dict.items():
            # h5py will convert nested lists/arrays to the correct dtype/shape.
            root[name][...] = np.array(array)  # type: ignore[assignment]

    print(f"Saving: {time.time() - t0:.1f} secs")

    robot_shutdown()
    return True


def main(args) -> None:
    """
    Entry point for follower-only episode recording.
    """
    torque_base = bool(args.get("enable_base_torque", False))
    robot_base = str(args.get("robot", ""))

    # These flags control whether extra torque-related observations are recorded
    get_gravity_torque = bool(args.get("gravity_torque", False))
    get_dynamics_torque = bool(args.get("dynamics_torque", False))

    # If set, follower gravity compensation is used instead of torque-off to make
    # the follower arm(s) feel weightless while you move them.
    use_follower_gravity_compensation = bool(
        args.get("follower_gravity_compensation", True)
    )

    base_path = Path(__file__).resolve().parent.parent / "config"

    # Load robot and task configurations
    config = load_yaml_file("robot", robot_base, str(base_path)).get("robot", {})
    task_config = load_yaml_file("task", base_path=str(base_path))
    task = task_config["tasks"].get(args.get("task_name"))

    if task is None:
        raise RuntimeError(
            f"Task '{args.get('task_name')}' not found in tasks configuration."
        )

    dataset_dir = os.path.expanduser(task.get("dataset_dir"))
    if not os.path.exists(dataset_dir):
        os.makedirs(dataset_dir)
    max_timesteps = task.get("episode_len")

    # Determine episode index (auto-index if not provided)
    episode_idx_arg = args.get("episode_idx")
    if episode_idx_arg is not None:
        episode_idx = int(episode_idx_arg)
    else:
        episode_idx = get_auto_index(dataset_dir)

    # Confirm overwrite behavior
    overwrite = check_episode_index(dataset_dir=dataset_dir, episode_idx=episode_idx)
    if not overwrite:
        return

    dataset_name = f"episode_{episode_idx}"
    print(f"{dataset_name}\n")

    exposure_arg = args.get("exposure", None)
    exposure = int(exposure_arg) if exposure_arg is not None else None

    # Retry capture if the frequency check fails
    while True:
        is_healthy = capture_one_episode_no_leader(
            max_timesteps=max_timesteps,
            dataset_dir=dataset_dir,
            dataset_name=dataset_name,
            overwrite=overwrite,
            torque_base=torque_base,
            use_follower_gravity_compensation=use_follower_gravity_compensation,
            config=config,
            get_gravity_torque=get_gravity_torque,
            get_dynamics_torque=get_dynamics_torque,
            exposure=exposure,
        )
        if is_healthy:
            break


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Record follower-only episodes with torque-disabled follower arm(s)."
    )

    parser.add_argument(
        "-t",
        "--task_name",
        action="store",
        type=str,
        help="Task name to specify the teleoperation task.",
        required=True,
    )

    parser.add_argument(
        "--episode_idx",
        action="store",
        type=int,
        help="Episode index to name the dataset file. Auto-generated if not provided.",
        default=None,
        required=False,
    )

    parser.add_argument(
        "-b",
        "--enable_base_torque",
        action="store_true",
        help=(
            "Enable base torque for mobile robots during recording. "
            "Allows joystick control or other manual methods."
        ),
    )

    parser.add_argument(
        "-r",
        "--robot",
        action="store",
        type=str,
        help="Robot setup configuration (e.g., aloha_solo, aloha_stationary, aloha_mobile).",
        required=True,
    )

    parser.add_argument(
        "--gravity_torque",
        action="store_true",
        help="Enable storing follower gravity torque information.",
    )

    parser.add_argument(
        "--dynamics_torque",
        action="store_true",
        help=(
            "Enable storing follower dynamics torque information "
            "(includes kinetic friction, static friction, dither speeds, no_load_currents)."
        ),
    )

    parser.add_argument(
        "--exposure",
        action="store",
        type=int,
        help="Camera exposure value in microseconds. If provided, sets exposure for all cameras.",
        default=None,
        required=False,
    )

    main(vars(parser.parse_args()))

