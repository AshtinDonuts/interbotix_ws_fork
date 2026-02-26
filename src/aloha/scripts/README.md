# Overview of scripts

## Basic Usage
### Source ROS and activate Conda Env
```
conda activate interbotix
source_trossen
```
### Bring up ALOHA
Make sure you get the get_dynamics_torque argument passed.
```
ros2 launch aloha aloha_bringup.launch.py robot:=aloha_solo get_dynamics_torque:=true
```

### Record Episodes with dynamics torques

**The recommended run script is:**
```
bash auto_record_overlay.sh soap_push 20 aloha_solo
bash auto_record_overlay_no_leader.sh usb_plug_hand 20 aloha_solo
```

Other options:
```
python3 record_episodes.py \
  --task_name soap_push \
  --robot aloha_solo --dynamics_torque
```

By default it already includes the flag for recording dynamics torques
```
bash auto_record.sh plug_insert 30 aloha_solo
bash auto_record.sh plug_insert 30 aloha_solo
```

### Record episodes with no leader

This one also has the overlay.
```
python3 record_episodes_no_leader.py \
  --task_name aloha_solo_dummy \
  --robot aloha_solo --dynamics_torque
```

python3 record_episodes_no_leader.py \
  --task_name usb_plug_hand \
  --robot aloha_solo --dynamics_torque