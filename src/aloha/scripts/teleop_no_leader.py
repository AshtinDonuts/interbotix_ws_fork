#!/usr/bin/env python3

"""
Follower-only teleoperation using the default Interbotix gravity compensation.

This script assumes:
- The ALOHA bringup launch file has already been started, e.g.:
    ros2 launch aloha aloha_bringup.launch.py robot:=aloha_solo

- Follower arms are defined in the robot YAML under `follower_arms`.
- The standard `interbotix_gravity_compensation` nodes are launched for each follower arm
  (typically handled by `aloha_bringup.launch.py` when `use_gravity_compensation:=true`).

Behavior:
- Moves all follower arms to a safe start pose.
- Closes follower grippers.
- Enables gravity compensation on follower arms via the default
  `interbotix_gravity_compensation` package (through
  `InterbotixGravityCompensationInterface`).
- Keeps the node alive so the arms remain in compensated mode until you press Ctrl+C.
"""

import argparse
import time
from pathlib import Path
from typing import Dict

from aloha.robot_utils import (
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    START_ARM_POSE,
    enable_gravity_compensation,
    disable_gravity_compensation,
    load_yaml_file,
    move_arms,
    move_grippers,
    torque_on,
)
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


def move_followers_to_start(robots: Dict[str, InterbotixManipulatorXS], dt: float) -> None:
    """
    Move all follower arms to the standard START_ARM_POSE and close their grippers.
    """
    follower_bots = {
        name: bot for name, bot in robots.items() if "follower" in name
    }

    if not follower_bots:
        raise RuntimeError(
            "No follower robots found in configuration. "
            "Check the robot YAML 'follower_arms' section."
        )

    # Ensure follower arm torques are on before moving.
    for bot in follower_bots.values():
        torque_on(bot)

    print("Moving follower arm(s) to start pose...")
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms(
        bot_list=list(follower_bots.values()),
        target_pose_list=[start_arm_qpos] * len(follower_bots),
        moving_time=4.0,
        dt=dt,
    )

    # Close follower grippers so the arm is in a compact, safe configuration.
    move_grippers(
        list(follower_bots.values()),
        [FOLLOWER_GRIPPER_JOINT_CLOSE] * len(follower_bots),
        moving_time=0.5,
        dt=dt,
    )


def enable_follower_gravity_compensation(
    robots: Dict[str, InterbotixManipulatorXS],
) -> None:
    """
    Enable gravity compensation on follower arms using the default
    `interbotix_gravity_compensation` package.

    This uses the existing `InterbotixGravityCompensationInterface`, which talks to the
    `/gravity_compensation_enable` service provided by the underlying C++ node in each
    follower namespace.
    """
    follower_bots = {
        name: bot for name, bot in robots.items() if "follower" in name
    }

    if not follower_bots:
        raise RuntimeError(
            "No follower robots found when enabling gravity compensation."
        )

    print("Enabling follower gravity compensation on:")
    for name, bot in follower_bots.items():
        print(f"  - {name}")
        enable_gravity_compensation(bot)


def disable_follower_gravity_compensation(
    robots: Dict[str, InterbotixManipulatorXS],
) -> None:
    """
    Disable follower gravity compensation on all follower arms.
    """
    follower_bots = {
        name: bot for name, bot in robots.items() if "follower" in name
    }
    for name, bot in follower_bots.items():
        print(f"Disabling follower gravity compensation on {name}...")
        try:
            disable_gravity_compensation(bot)
        except Exception as exc:  # noqa: BLE001
            print(f"  Warning: failed to disable gravity compensation on {name}: {exc}")


def main(args: dict) -> None:
    """
    Follower-only teleoperation entry point.

    - Loads the specified robot configuration.
    - Instantiates follower arms.
    - Moves them to a start pose.
    - Enables follower gravity compensation.
    - Keeps the script alive until interrupted.
    """
    robot_base = args.get("robot", "")

    if not robot_base:
        raise RuntimeError(
            "Robot configuration name must be provided via '--robot'. "
            "Example: --robot aloha_stationary"
        )

    base_path = Path(__file__).resolve().parent.parent / "config"
    config = load_yaml_file("robot", robot_base, str(base_path)).get("robot", {})

    dt = 1.0 / config.get("fps", 30)

    # Initialize shared Interbotix node and follower arms.
    node = create_interbotix_global_node("aloha")

    robots: Dict[str, InterbotixManipulatorXS] = {}

    for follower in config.get("follower_arms", []):
        robot_instance = InterbotixManipulatorXS(
            robot_model=follower["model"],
            robot_name=follower["name"],
            node=node,
            iterative_update_fk=False,
        )
        robots[follower["name"]] = robot_instance

    if not robots:
        raise RuntimeError(
            "No follower arms were instantiated. "
            "Check that 'follower_arms' is populated in the robot YAML."
        )

    robot_startup(node)

    try:
        move_followers_to_start(robots, dt)

        print("\nFollower-only teleoperation with gravity compensation.")
        print(
            "Ensure the follower arm(s) are in a safe pose and physically supported "
            "before enabling gravity compensation."
        )
        input(
            "Press ENTER to enable follower gravity compensation and begin teleop.\n"
            "Then gently move the follower arm(s) directly by hand.\n"
            "Press Ctrl+C in this terminal to stop.\n"
        )

        enable_follower_gravity_compensation(robots)
        print("\nFollower gravity compensation ENABLED.")
        print("Move the follower arm(s) by hand. Press Ctrl+C to stop.")

        # Keep the process alive while gravity compensation is active.
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Stopping follower teleop...")
    finally:
        disable_follower_gravity_compensation(robots)
        robot_shutdown(node)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Follower-only teleoperation using follower gravity compensation.\n"
        )
    )

    parser.add_argument(
        "-r",
        "--robot",
        action="store",
        type=str,
        required=True,
        help=(
            "Robot setup configuration (e.g., aloha_solo, aloha_stationary, "
            "aloha_mobile). Must match the YAML in 'config/robot/'."
        ),
    )

    main(vars(parser.parse_args()))

