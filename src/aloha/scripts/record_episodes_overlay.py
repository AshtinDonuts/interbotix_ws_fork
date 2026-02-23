#!/usr/bin/env python3

"""
Top-view overlay visualizer with OpenCV.
Opens an image at a specified path, allows user to draw lines over it, then saves the edited image.
If the image does not exist, it takes an image with the left_shoulder_cam.
"""

import argparse
import cv2
import numpy as np
import os
from pathlib import Path
import rclpy
import time
from enum import Enum

from aloha.robot_utils import ImageRecorder, load_yaml_file
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)


class HighlightColor(Enum):
    """
    Common highlighting colors in BGR format for OpenCV.
    
    Available colors:
    - CYAN: Bright cyan (255, 200, 0)
    - YELLOW: Bright yellow (0, 255, 255)
    - MAGENTA: Bright magenta/pink (255, 0, 255)
    - RED: Bright red (0, 0, 255)
    - GREEN: Bright green (0, 255, 0)
    - BLUE: Bright blue (255, 0, 0)
    - ORANGE: Orange (0, 165, 255)
    - LIME: Lime green (0, 255, 128)
    - PINK: Pink (203, 192, 255)
    - WHITE: White (255, 255, 255)
    - LIGHT_BLUE: Light blue (255, 144, 30)
    - PURPLE: Purple (128, 0, 128)
    """
    CYAN = (255, 200, 0)          # Bright cyan
    YELLOW = (0, 255, 255)         # Bright yellow
    MAGENTA = (255, 0, 255)        # Bright magenta/pink
    RED = (0, 0, 255)              # Bright red
    GREEN = (0, 255, 0)            # Bright green
    BLUE = (255, 0, 0)             # Bright blue
    ORANGE = (0, 165, 255)         # Orange
    LIME = (0, 255, 128)           # Lime green
    PINK = (203, 192, 255)         # Pink
    WHITE = (255, 255, 255)        # White
    LIGHT_BLUE = (255, 144, 30)    # Light blue
    PURPLE = (128, 0, 128)         # Purple
    
    def __iter__(self):
        """Allow unpacking the color tuple."""
        return iter(self.value)


class LineDrawer:
    """OpenCV-based line drawing interface."""
    
    @staticmethod
    def get_available_colors():
        """
        Get a list of available highlighting colors.
        
        :return: Dictionary mapping color names to BGR tuples.
        """
        return {color.name: color.value for color in HighlightColor}
    
    def __init__(self, image_path: str, output_path: str | None = None, line_color: HighlightColor | None = None):
        """
        Initialize the line drawer.
        
        :param image_path: Path to the final combined image (base + lines).
        :param output_path: Path to save the edited image. If None, overwrites the original.
        :param line_color: HighlightColor enum value for line color. If None, uses CYAN.
                          Available: CYAN, YELLOW, MAGENTA, RED, GREEN, BLUE, ORANGE, 
                          LIME, PINK, WHITE, LIGHT_BLUE, PURPLE
        """
        self.image_path = image_path
        self.output_path = output_path if output_path is not None else image_path
        
        # Derive base and lines paths from the main image path
        base_path = Path(image_path)
        self.base_image_path = str(base_path.parent / f"{base_path.stem}_base{base_path.suffix}")
        self.lines_image_path = str(base_path.parent / f"{base_path.stem}_lines{base_path.suffix}")
        
        self.drawing = False
        self.start_point = None
        self.base_image = None
        self.lines_overlay = None
        self.current_image = None
        self.display_image = None
        
        # Line drawing parameters
        # Available colors: HighlightColor.CYAN, YELLOW, MAGENTA, RED, GREEN, BLUE, 
        #                  ORANGE, LIME, PINK, WHITE, LIGHT_BLUE, PURPLE
        if line_color is None:
            line_color = HighlightColor.RED
        self.line_color = line_color.value
        self.line_thickness = 2  # Thinner lines
        self.line_opacity = 0.8  # 40% opacity for semi-transparent lines
        
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for drawing lines."""
        if self.current_image is None or self.lines_overlay is None:
            return
            
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_point = (x, y)
            
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing and self.start_point is not None:
                # Create a copy for preview
                self.display_image = self.current_image.copy()
                # Draw preview line with custom color and thickness
                cv2.line(self.display_image, self.start_point, (x, y), self.line_color, self.line_thickness)
                cv2.imshow('Top View Overlay', self.display_image)
                
        elif event == cv2.EVENT_LBUTTONUP:
            if self.drawing and self.start_point is not None:
                # Draw the final line on the lines overlay with custom color and thickness
                cv2.line(self.lines_overlay, self.start_point, (x, y), self.line_color, self.line_thickness)
                # Update the combined image with lower opacity for lines
                if self.base_image is not None and self.lines_overlay is not None:
                    self.current_image = cv2.addWeighted(self.base_image, 1.0, self.lines_overlay, self.line_opacity, 0)
                    self.display_image = self.current_image.copy()
                    cv2.imshow('Top View Overlay', self.display_image)
                self.drawing = False
                self.start_point = None
    
    def run(self, robot_config: str = "aloha_solo", camera_name: str = "camera_left_shoulder"):
        """
        Run the line drawing interface.
        
        :param robot_config: Robot configuration name for camera capture.
        :param camera_name: Camera name for camera capture.
        """
        # Create window first
        window_name = 'Top View Overlay'
        cv2.namedWindow(window_name)
        
        # Initialize camera for streaming
        print("\n=== Initializing Camera ===")
        node, image_recorder = initialize_camera(robot_config, camera_name)
        if node is None or image_recorder is None:
            print("Failed to initialize camera.")
            cv2.destroyAllWindows()
            return False
        
        # Load accumulated lines overlay from previous episodes (if exists) - before capture
        lines_overlay_for_stream = None
        if os.path.exists(self.lines_image_path):
            lines_overlay_for_stream = cv2.imread(self.lines_image_path)
            if lines_overlay_for_stream is not None:
                print(f"Loaded accumulated lines from previous episodes for preview")
        
        print("\n=== Waiting for Capture ===")
        print("Press 'c' to capture base image from camera")
        print("Press 'q' to quit")
        print("You can see the live camera stream with accumulated lines overlaid")
        print("===========================\n")
        
        # Stream camera with accumulated lines overlaid while waiting for capture
        base_captured = False
        while not base_captured:
            # Get current camera frame
            frame = get_camera_frame(image_recorder, camera_name)
            
            if frame is not None:
                # Overlay accumulated lines on live stream
                if lines_overlay_for_stream is not None:
                    # Resize lines overlay if dimensions don't match
                    h_frame, w_frame = frame.shape[:2]
                    h_lines, w_lines = lines_overlay_for_stream.shape[:2]
                    if h_frame != h_lines or w_frame != w_lines:
                        lines_overlay_resized = cv2.resize(lines_overlay_for_stream, (w_frame, h_frame))
                    else:
                        lines_overlay_resized = lines_overlay_for_stream
                    # Combine live frame with accumulated lines (using lower opacity)
                    display_frame = cv2.addWeighted(frame, 1.0, lines_overlay_resized, 0.4, 0)
                else:
                    display_frame = frame
                
                # Add text overlay with instructions
                cv2.putText(display_frame, "Press 'c' to capture", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_frame, "Press 'q' to quit", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                cv2.imshow(window_name, display_frame)
            else:
                # Show error message if frame not available
                error_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(error_image, "Waiting for camera...", (150, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow(window_name, error_image)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('c'):
                # Capture current frame as base image
                frame = get_camera_frame(image_recorder, camera_name)
                if frame is None:
                    print("Failed to capture frame. Try again.")
                    continue
                
                self.base_image = frame.copy()
                
                # Save the new base image
                cv2.imwrite(self.base_image_path, self.base_image)
                print(f"New base image captured and saved to {self.base_image_path}")
                base_captured = True
                
            elif key == ord('q') or key == 27:  # 'q' or ESC
                print("Exiting without capturing.")
                robot_shutdown()
                cv2.destroyAllWindows()
                return False
        
        # Shutdown camera streaming (we'll use the captured base image now)
        robot_shutdown()
        
        # Use the lines overlay we loaded for streaming, or create a new one
        if self.base_image is None:
            print("Error: Base image is None")
            cv2.destroyAllWindows()
            return False
            
        if lines_overlay_for_stream is not None:
            # Reuse the loaded overlay, but resize if dimensions don't match base image
            h_base, w_base = self.base_image.shape[:2]
            h_lines, w_lines = lines_overlay_for_stream.shape[:2]
            if h_base != h_lines or w_base != w_lines:
                print(f"Resizing lines overlay to match new base image dimensions ({w_base}x{h_base})")
                self.lines_overlay = cv2.resize(lines_overlay_for_stream, (w_base, h_base))
            else:
                self.lines_overlay = lines_overlay_for_stream.copy()
            print(f"Using accumulated lines from previous episodes")
        else:
            # Create a blank overlay with same dimensions as base
            h, w = self.base_image.shape[:2]
            self.lines_overlay = np.zeros((h, w, 3), dtype=np.uint8)
            print(f"No previous lines found - starting with blank overlay")
        
        # Combine NEW base image with accumulated lines overlay (using lower opacity)
        if self.base_image is not None and self.lines_overlay is not None:
            self.current_image = cv2.addWeighted(self.base_image, 1.0, self.lines_overlay, self.line_opacity, 0)
            self.display_image = self.current_image.copy()
        else:
            print("Error: Failed to initialize images properly")
            cv2.destroyAllWindows()
            return False
        
        # Set mouse callback for drawing
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        # Display drawing instructions
        print("\n=== Drawing Instructions ===")
        print("Left-click and drag to draw lines")
        print("Press 's' to save and exit")
        print("Press 'q' or ESC to exit without saving")
        print("Press 'r' to reset (clear only new lines, keep previous lines)")
        print("===========================\n")
        
        # Main loop
        while True:
            cv2.imshow(window_name, self.display_image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s'):
                # Save the lines overlay and combined image
                cv2.imwrite(self.lines_image_path, self.lines_overlay)
                cv2.imwrite(self.output_path, self.current_image)
                print(f"Lines overlay saved to {self.lines_image_path}")
                print(f"Combined image saved to {self.output_path}")
                break
            elif key == ord('q') or key == 27:  # 'q' or ESC
                print("Exiting without saving.")
                break
            elif key == ord('r'):
                # Reset: clear only the new lines drawn in this session
                # Reload the saved lines overlay (removes unsaved new lines drawn this episode)
                if os.path.exists(self.lines_image_path):
                    reloaded_lines = cv2.imread(self.lines_image_path)
                    if reloaded_lines is not None and self.base_image is not None:
                        # Resize if needed
                        h_base, w_base = self.base_image.shape[:2]
                        h_lines, w_lines = reloaded_lines.shape[:2]
                        if h_base != h_lines or w_base != w_lines:
                            reloaded_lines = cv2.resize(reloaded_lines, (w_base, h_base))
                        self.lines_overlay = reloaded_lines
                        self.current_image = cv2.addWeighted(self.base_image, 1.0, self.lines_overlay, self.line_opacity, 0)
                        self.display_image = self.current_image.copy()
                        cv2.imshow(window_name, self.display_image)
                        print("Reset to saved lines (cleared unsaved new lines from this episode).")
                    else:
                        # If reload failed, clear all lines
                        h, w = self.base_image.shape[:2]
                        self.lines_overlay = np.zeros((h, w, 3), dtype=np.uint8)
                        self.current_image = self.base_image.copy()
                        self.display_image = self.current_image.copy()
                        cv2.imshow(window_name, self.display_image)
                        print("Reset: cleared all lines.")
                else:
                    # No saved lines, just show base
                    h, w = self.base_image.shape[:2]
                    self.lines_overlay = np.zeros((h, w, 3), dtype=np.uint8)
                    self.current_image = self.base_image.copy()
                    self.display_image = self.current_image.copy()
                    cv2.imshow(window_name, self.display_image)
                    print("Reset: cleared all lines.")
        
        cv2.destroyAllWindows()
        return True


def get_camera_frame(image_recorder, camera_name: str):
    """
    Get a single frame from the camera.
    
    :param image_recorder: ImageRecorder instance.
    :param camera_name: Name of the camera to use.
    :return: Camera frame as numpy array, or None if failed.
    """
    images = image_recorder.get_images()
    if camera_name in images and images[camera_name] is not None:
        return images[camera_name]
    return None


def initialize_camera(robot_config: str = "aloha_solo", camera_name: str = "camera_left_shoulder"):
    """
    Initialize ROS node and ImageRecorder for camera streaming.
    
    :param robot_config: Robot configuration name (e.g., 'aloha_solo').
    :param camera_name: Name of the camera to use (default: 'camera_left_shoulder').
    :return: Tuple of (node, image_recorder) or (None, None) if failed.
    """
    # Initialize ROS if not already initialized
    if not rclpy.ok():
        rclpy.init()
    
    # Initialize ROS node
    node = create_interbotix_global_node("aloha_overlay")
    robot_startup(node)
    
    try:
        # Load robot configuration
        base_path = Path(__file__).resolve().parent.parent / "config"
        config_dict = load_yaml_file("robot", robot_config, str(base_path))
        config = config_dict.get('robot', {})
        
        # Create ImageRecorder
        image_recorder = ImageRecorder(node=node, config=config)
        
        # Wait for camera to initialize
        print(f"Waiting for {camera_name} to initialize...")
        time.sleep(1.0)  # Give camera time to start
        
        # Verify we can get frames
        max_attempts = 30
        for attempt in range(max_attempts):
            frame = get_camera_frame(image_recorder, camera_name)
            if frame is not None:
                print(f"Camera {camera_name} ready for streaming")
                return node, image_recorder
            time.sleep(0.1)
        
        print(f"Failed to initialize {camera_name} after {max_attempts} attempts")
        robot_shutdown()
        return None, None
        
    except Exception as e:
        print(f"Error initializing camera: {e}")
        import traceback
        traceback.print_exc()
        robot_shutdown()
        return None, None


def capture_camera_image(robot_config: str = "aloha_solo", camera_name: str = "camera_left_shoulder"):
    """
    Capture an image from the specified camera (legacy function for single capture).
    
    :param robot_config: Robot configuration name (e.g., 'aloha_solo').
    :param camera_name: Name of the camera to use (default: 'camera_left_shoulder').
    :return: Captured image as numpy array, or None if failed.
    """
    node, image_recorder = initialize_camera(robot_config, camera_name)
    if node is None or image_recorder is None:
        return None
    
    try:
        frame = get_camera_frame(image_recorder, camera_name)
        return frame
    finally:
        robot_shutdown()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Top-view overlay visualizer. Draw lines on images or capture from camera."
    )
    
    parser.add_argument(
        "-i",
        "--image_path",
        type=str,
        required=True,
        help="Path to the image file. If it doesn't exist, an image will be captured from the camera.",
    )
    
    parser.add_argument(
        "-o",
        "--output_path",
        type=str,
        default=None,
        help="Path to save the edited image. If not specified, overwrites the input image.",
    )
    
    parser.add_argument(
        "-r",
        "--robot",
        type=str,
        default="aloha_solo",
        help="Robot configuration name (default: 'aloha_solo').",
    )
    
    parser.add_argument(
        "-c",
        "--camera",
        type=str,
        default="camera_left_shoulder",
        help="Camera name to use if image doesn't exist (default: 'camera_left_shoulder').",
    )
    
    parser.add_argument(
        "--color",
        type=str,
        default="CYAN",
        choices=[color.name for color in HighlightColor],
        help=f"Line color for drawing. Available: {', '.join([c.name for c in HighlightColor])} (default: CYAN).",
    )
    
    args = parser.parse_args()
    
    # Get the color enum from the argument
    line_color = HighlightColor[args.color.upper()]
    
    # Create and run the line drawer
    drawer = LineDrawer(args.image_path, args.output_path, line_color=line_color)
    success = drawer.run(args.robot, args.camera)
    
    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
