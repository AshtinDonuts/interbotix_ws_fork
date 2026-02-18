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

```
python3 record_episodes.py \
  --task_name aloha_solo_dummy \
  --robot aloha_solo --dynamics_torque
```

```
bash auto_record.sh plug_insert 1 aloha_solo --dynamics_torque
```