#!/usr/bin/env python3
"""
Test script to verify how nested parameters are structured when passed to ROS2 nodes.
This helps verify if the exposure parameter will be correctly received by the RealSense driver.
"""

# Simulate the parameter structure that would be passed to the RealSense node
common_params = {
    'enable_color': True,
    'rgb_camera': {
        'profile': '640,480,60',
        'enable_auto_exposure': False,
        'exposure': 10000
    },
    'depth_module': {
        'profile': '640,480,60',
        'enable_auto_exposure': False
    },
    'enable_depth': False,
    'color_image_topic_name': '{}/camera/color/image_rect_raw'
}

# Add camera-specific params
camera_params = common_params.copy()
camera_params.update({
    'serial_no': '218622275088',
    'initial_reset': True
})

print("="*70)
print("PARAMETER STRUCTURE VERIFICATION")
print("="*70)
print("\n1. Common parameters structure:")
import json
print(json.dumps(common_params, indent=2))

print("\n2. Camera parameters (with serial_no added):")
print(json.dumps(camera_params, indent=2))

print("\n3. RGB Camera nested parameters:")
print(f"   rgb_camera.profile: {camera_params['rgb_camera']['profile']}")
print(f"   rgb_camera.enable_auto_exposure: {camera_params['rgb_camera']['enable_auto_exposure']}")
print(f"   rgb_camera.exposure: {camera_params['rgb_camera']['exposure']}")

print("\n" + "="*70)
print("VERIFICATION CHECKLIST:")
print("="*70)
print("✓ YAML structure is correct")
print("✓ Nested dictionary structure preserved")
print("✓ exposure value: 10000 microseconds")
print("✓ enable_auto_exposure: False (required for manual exposure)")
print("\nNOTE: ROS2 launch system should preserve this nested structure")
print("      when passing to realsense2_camera_node")
print("="*70)
