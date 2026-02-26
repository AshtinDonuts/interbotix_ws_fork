#!/usr/bin/env python3

"""
- Test script to initialize a core for an XS follower arm and receive state information.

NOTE: current no arm movement is allowed. This is a TODO feature.

## USAGE
python3 TEST_core_functions.py

python3 TEST_core_functions.py --update_rate 2.0 --duration 3.0

"""

import argparse
import rclpy
import time
from pathlib import Path

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_startup,
    robot_shutdown,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from aloha.robot_utils import load_yaml_file, torque_off, torque_on


def main():
    """Initialize follower arm core and continuously print joint state information."""
    parser = argparse.ArgumentParser(
        description="Test script to initialize follower arm core and receive joint states."
    )
    parser.add_argument(
        "-r",
        "--robot",
        type=str,
        default="aloha_solo",
        help="Robot configuration name (e.g., aloha_solo, aloha_stationary). Default: aloha_solo",
    )
    parser.add_argument(
        "--follower_name",
        type=str,
        default=None,
        help="Specific follower arm name to use. If not provided, uses the first follower from config.",
    )
    parser.add_argument(
        "--update_rate",
        type=float,
        default=10.0,
        help="Rate (Hz) at which to print joint state information. Default: 10.0",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Duration in seconds to run the test. If not provided, runs indefinitely until Ctrl+C.",
    )
    parser.add_argument(
        "--disable_torque",
        action="store_true",
        help="Turn off motor torques for the follower arm. By default, torque state is unchanged.",
    )

    args = parser.parse_args()

    # Initialize ROS node
    print("Initializing ROS node...")
    node = create_interbotix_global_node("test_core_functions")
    robot_startup(node)

    try:
        # Load robot configuration
        base_path = Path(__file__).resolve().parent.parent / "config"
        config = load_yaml_file("robot", args.robot, base_path).get("robot", {})

        # Get follower arm configuration
        follower_arms = config.get("follower_arms", [])
        if not follower_arms:
            raise ValueError(f"No follower arms found in robot configuration '{args.robot}'")

        # Select follower arm
        if args.follower_name:
            follower_config = next(
                (f for f in follower_arms if f["name"] == args.follower_name), None
            )
            if follower_config is None:
                raise ValueError(
                    f"Follower arm '{args.follower_name}' not found in configuration. "
                    f"Available followers: {[f['name'] for f in follower_arms]}"
                )
        else:
            follower_config = follower_arms[0]
            print(f"Using first follower arm from config: {follower_config['name']}")

        # Initialize follower arm (this creates the core and subscribes to joint states)
        print(f"Initializing follower arm: {follower_config['name']} (model: {follower_config['model']})...")
        follower_bot = InterbotixManipulatorXS(
            robot_model=follower_config["model"],
            robot_name=follower_config["name"],
            node=node,
            iterative_update_fk=False,
        )

        # Wait a moment for joint states to be received
        print("Waiting for joint states...")
        time.sleep(1.0)

        # Verify joint states are being received
        try:
            test_positions = follower_bot.arm.get_joint_positions()
            print(f"✓ Successfully receiving joint states! ({len(test_positions)} arm joints)")
        except Exception as e:
            print(f"✗ Error receiving joint states: {e}")
            print("Make sure the robot is powered on and the xs_sdk node is running.")
            return

        # Optionally turn off motor torques so we can move the arm around
        # TODO  -  figure out how to do this
        #       -  must follower be paired with leader during setup?
        if args.disable_torque:
            raise NotImplementedError
            print("Turning off motor torques for follower arm...")
            torque_off(follower_bot)
            print("✓ Motor torques disabled")
        else:
            print("Motor torque state unchanged (using current robot state)")

        # Get joint names for display
        joint_names = follower_bot.arm.group_info.joint_names
        num_arm_joints = len(joint_names)

        print(f"\n{'='*80}")
        print(f"Follower Arm: {follower_config['name']}")
        print(f"Model: {follower_config['model']}")
        print(f"Arm Joints: {num_arm_joints}")
        print(f"Joint Names: {', '.join(joint_names)}")
        print(f"Update Rate: {args.update_rate} Hz")
        print(f"Torque Disabled: {args.disable_torque}")
        print(f"{'='*80}\n")

        # Main loop: continuously read and print joint state information
        dt = 1.0 / args.update_rate
        start_time = time.time()
        iteration = 0

        try:
            while rclpy.ok():
                iteration += 1
                current_time = time.time()
                elapsed_time = current_time - start_time

                # Check duration limit
                if args.duration is not None and elapsed_time >= args.duration:
                    print(f"\nDuration limit ({args.duration}s) reached. Stopping.")
                    break

                # Get joint state information
                positions = follower_bot.arm.get_joint_positions()
                velocities = follower_bot.arm.get_joint_velocities()
                # efforts = follower_bot.arm.get_joint_efforts()
                # gripper_position = follower_bot.gripper.get_gripper_position()

                # Print joint state information
                print(f"\n--- Iteration {iteration} | Time: {elapsed_time:.2f}s ---")
                print(f"Arm Joint Positions (rad): {[f'{p:.4f}' for p in positions]}")
                # print(f"Arm Joint Velocities (rad/s): {[f'{v:.4f}' for v in velocities]}")
                # print(f"Arm Joint Efforts: {[f'{e:.2f}' for e in efforts]}")
                # print(f"Gripper Position: {gripper_position:.4f}")


                # Get gravity torque information
                gravity_torques = follower_bot.arm.get_gravity_torques()

                # Print Gravity Torque Information
                print(f"Arm Gravity Torques (Nm): {[f'{t:.4f}' for t in gravity_torques]}")


                # Sleep to maintain update rate
                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n\nInterrupted by user. Shutting down...")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("Shutting down...")
        robot_shutdown()


if __name__ == "__main__":
    main()

