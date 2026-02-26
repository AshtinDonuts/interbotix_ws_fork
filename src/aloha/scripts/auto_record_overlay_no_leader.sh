# AUTO RECORD EPISODE WITH IMAGE OVERLAY CAPABILITY

ROS_DISTRO=humble

ROS_SETUP_PATH=/opt/ros/$ROS_DISTRO/setup.bash
WORKSPACE_SETUP_PATH=$HOME/interbotix_ws/install/setup.bash
RECORD_EPISODES="$HOME/interbotix_ws/src/aloha/scripts/record_episodes_no_leader.py"
RECORD_OVERLAY="$HOME/interbotix_ws/src/aloha/scripts/record_episodes_overlay.py"

source $ROS_SETUP_PATH || exit 1
source $WORKSPACE_SETUP_PATH || exit 1

print_usage() {
  echo "USAGE:"
  echo "auto_record_overlay_no_leader.sh task num_episodes robot_name [-b, --enable_base_torque] [-g, --gravity_compensation] --dynamics_torque"
}

nargs="$#"

if [ $nargs -lt 3 ]; then
  echo "Passed incorrect number of arguments"
  print_usage
  exit 1
fi

if [ "$2" -lt 0 ]; then
  echo "# of episodes not valid"
  exit 1
fi

# Get dataset directory from task config
TASK_NAME=$1
ROBOT_NAME=$3
CONFIG_PATH="$HOME/interbotix_ws/src/aloha/config"
DATASET_DIR=$(python3 -c "
import yaml
from pathlib import Path
config_path = Path('$CONFIG_PATH') / 'tasks_config.yaml'
with open(config_path, 'r') as f:
    task_config = yaml.safe_load(f)
task = task_config['tasks'].get('$TASK_NAME')
if task is None:
    print('ERROR: Task $TASK_NAME not found in config', file=__import__('sys').stderr)
    exit(1)
dataset_dir = task.get('dataset_dir')
if dataset_dir is None:
    print('ERROR: dataset_dir not found for task $TASK_NAME', file=__import__('sys').stderr)
    exit(1)
import os
print(os.path.expanduser(dataset_dir))
")

if [ $? -ne 0 ]; then
  echo "Failed to get dataset directory from task config"
  exit 1
fi

# Create dataset directory if it doesn't exist
mkdir -p "$DATASET_DIR"

# Use a single shared overlay image that accumulates drawings across episodes
OVERLAY_IMAGE="$DATASET_DIR/overlay.jpg"

echo "Task: $TASK_NAME"
echo "Dataset directory: $DATASET_DIR"
echo "Overlay image (shared across all episodes): $OVERLAY_IMAGE"
echo ""

for (( i=0; i<$2; i++ ))
do
  echo "========================================="
  echo "Recording episode $((i+1)) of $2"
  echo "========================================="
  
  # Run overlay visualizer before recording
  # The overlay image will be created on first episode (from camera if needed)
  # Subsequent episodes will load the existing image and add to it
  if [ $i -eq 0 ]; then
    echo "Creating overlay image (will capture from camera if image doesn't exist)..."
  else
    echo "Loading existing overlay image to add more drawings..."
  fi
  echo "Overlay image: $OVERLAY_IMAGE"
  python3 "$RECORD_OVERLAY" -i "$OVERLAY_IMAGE" -r "$ROBOT_NAME"
  OVERLAY_EXIT_CODE=$?
  
  if [ $OVERLAY_EXIT_CODE -ne 0 ]; then
    echo "Overlay visualizer failed or was cancelled. Skipping this episode."
    read -p "Continue to next episode? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
      echo "Stopping recording."
      exit 1
    fi
    continue
  fi
  
  echo "Starting episode recording (auto-indexing will determine episode number)..."
  python3 "$RECORD_EPISODES" --task $TASK_NAME -r $ROBOT_NAME $4 $5
  if [ $? -ne 0 ]; then
    echo "Failed to execute command. Returning"
    exit 1
  fi
done
