# Verification: Camera Exposure Settings

## ✅ Configuration Check

### 1. YAML Configuration Files
All three robot configuration files have been updated with:
- `rgb_camera.exposure: 10000` (microseconds)
- `rgb_camera.enable_auto_exposure: false` (required for manual exposure)

**Files verified:**
- ✅ `src/aloha/config/robot/aloha_solo.yaml`
- ✅ `src/aloha/config/robot/aloha_stationary.yaml`
- ✅ `src/aloha/config/robot/aloha_mobile.yaml`

### 2. Parameter Structure
The nested dictionary structure is correct:
```yaml
rgb_camera:
  profile: '640,480,60'
  enable_auto_exposure: false
  exposure: 10000
```

### 3. Launch File Parameter Passing
The launch file (`aloha_bringup.launch.py`) correctly:
- Loads `common_parameters` from YAML
- Merges with camera-specific settings (serial_no)
- Passes nested dictionary to `realsense2_camera_node`

## ⚠️ Potential Issues to Verify

### Issue 1: ROS2 Parameter Format
ROS2 should preserve nested dictionaries when passed as `parameters=[dict]`, but verify:
- The RealSense driver may expect parameters in a specific format
- Nested parameters might need to be flattened with dot notation

### Issue 2: Parameter Application Timing
According to RealSense documentation:
- Exposure must be set **before** streaming begins
- Auto-exposure must be disabled **before** setting manual exposure
- The current config has `enable_auto_exposure: false` which is correct

## 🔍 How to Verify Settings Are Applied

### Method 1: Check ROS2 Parameters (After Launch)
```bash
# List all parameters for a camera
ros2 param list /camera_wrist_left/camera

# Get specific exposure parameter
ros2 param get /camera_wrist_left/camera rgb_camera.exposure

# Should return: Integer value is: 10000
```

### Method 2: Use Verification Script
```bash
cd /home/khw/interbotix_ws
source install/setup.bash
ros2 run aloha verify_camera_params.py camera_wrist_left camera_wrist_right
```

### Method 3: Check Camera Node Logs
When launching, check the RealSense camera node output for:
- Parameter loading messages
- Exposure setting confirmation
- Any errors about parameter format

### Method 4: Visual Verification
- Launch cameras and check image brightness
- Images should have consistent exposure (not auto-adjusting)
- Use RealSense Viewer to confirm hardware exposure value

## 🛠️ If Settings Are Not Applied

### Solution 1: Flatten Parameters (if nested structure doesn't work)
If the RealSense driver doesn't accept nested dictionaries, modify the launch file to flatten parameters:

```python
# In aloha_bringup.launch.py, flatten nested parameters
def flatten_params(params, prefix=''):
    flattened = {}
    for key, value in params.items():
        new_key = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            flattened.update(flatten_params(value, new_key))
        else:
            flattened[new_key] = value
    return flattened

# Then use:
camera_params_flat = flatten_params(camera_params)
```

### Solution 2: Use ParameterFile
Instead of passing dictionary directly, create a temporary YAML file:

```python
from launch_ros.parameter_descriptions import ParameterFile
import tempfile
import yaml

# Create temp file with parameters
with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
    yaml.dump({'/**': {'ros__parameters': camera_params}}, f)
    temp_file = f.name

# Use ParameterFile
parameters=[ParameterFile(temp_file)]
```

## 📋 Verification Checklist

- [ ] YAML files have `exposure: 10000` under `rgb_camera`
- [ ] YAML files have `enable_auto_exposure: false`
- [ ] Launch file loads and passes parameters correctly
- [ ] Camera nodes start without parameter errors
- [ ] ROS2 parameter server shows correct exposure value
- [ ] Camera images have consistent exposure (not auto-adjusting)
- [ ] RealSense Viewer confirms hardware exposure is 10000

## 🚀 Next Steps

1. **Launch the system:**
   ```bash
   ros2 launch aloha aloha_bringup.launch.py robot:=aloha_stationary use_cameras:=true
   ```

2. **Verify parameters:**
   ```bash
   ros2 param get /camera_wrist_left/camera rgb_camera.exposure
   ```

3. **If parameters are not set correctly**, implement Solution 1 or 2 above.

4. **Test with recording:**
   ```bash
   python3 src/aloha/scripts/record_episodes.py -r aloha_stationary -t <task_name>
   ```
