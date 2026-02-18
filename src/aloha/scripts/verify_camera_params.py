#!/usr/bin/env python3
"""
Script to verify camera parameters are correctly set in ROS2 parameter server.
Run this after launching the camera nodes to check if exposure settings are applied.
"""

import rclpy
from rclpy.node import Node
import sys


class ParameterChecker(Node):
    def __init__(self):
        super().__init__('parameter_checker')
        
    def check_camera_params(self, camera_name: str):
        """Check parameters for a specific camera namespace."""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Checking parameters for: {camera_name}")
        self.get_logger().info(f"{'='*60}")
        
        # Check if node exists
        try:
            # Try to get a parameter to see if node exists
            client = self.create_client(
                type(rclpy.parameter.Parameter),
                f'/{camera_name}/get_parameters'
            )
            
            # List all parameters for this camera
            param_names = [
                'rgb_camera.exposure',
                'rgb_camera.enable_auto_exposure',
                'rgb_camera.profile',
                'enable_color',
            ]
            
            self.get_logger().info(f"\nAttempting to read parameters from /{camera_name}/camera...")
            
            # Use ros2 param CLI equivalent approach
            import subprocess
            result = subprocess.run(
                ['ros2', 'param', 'list', f'/{camera_name}/camera'],
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                self.get_logger().info(f"Parameters found:\n{result.stdout}")
                
                # Try to get specific parameters
                for param in param_names:
                    result = subprocess.run(
                        ['ros2', 'param', 'get', f'/{camera_name}/camera', param],
                        capture_output=True,
                        text=True
                    )
                    if result.returncode == 0:
                        self.get_logger().info(f"  {param}: {result.stdout.strip()}")
                    else:
                        self.get_logger().warn(f"  {param}: NOT FOUND or ERROR")
            else:
                self.get_logger().error(f"Camera node /{camera_name}/camera not found!")
                self.get_logger().error(f"Error: {result.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"Error checking parameters: {e}")


def main():
    rclpy.init()
    
    checker = ParameterChecker()
    
    # Get camera names from command line or use defaults
    if len(sys.argv) > 1:
        camera_names = sys.argv[1:]
    else:
        # Default camera names (adjust based on your config)
        camera_names = [
            'camera_wrist_left',
            'camera_wrist_right',
            'camera_high',
            'camera_low',
        ]
    
    for camera_name in camera_names:
        checker.check_camera_params(camera_name)
        rclpy.spin_once(checker, timeout_sec=0.1)
    
    checker.get_logger().info("\n" + "="*60)
    checker.get_logger().info("Verification complete!")
    checker.get_logger().info("="*60)
    checker.get_logger().info("\nTo manually check parameters, run:")
    checker.get_logger().info("  ros2 param list /<camera_name>/camera")
    checker.get_logger().info("  ros2 param get /<camera_name>/camera rgb_camera.exposure")
    
    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
