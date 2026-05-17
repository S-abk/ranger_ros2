# Agent onboarding guide (sim_jazzy)

This repository contains both runtime code and large simulation assets.  
To avoid context bloat, follow this order and scope.

## Read-first order (minimal context)
1. `/home/runner/work/ranger_ros2/ranger_ros2/README.md`
2. `/home/runner/work/ranger_ros2/ranger_ros2/ranger_mini_v3_sim/README.md`
3. `/home/runner/work/ranger_ros2/ranger_ros2/ranger_msgs/msg/*.msg`
4. `/home/runner/work/ranger_ros2/ranger_ros2/ranger_mini_v3_sim/launch/gazebo_full.launch.py`
5. `/home/runner/work/ranger_ros2/ranger_ros2/ranger_mini_v3_sim_messenger/launch/messenger.launch.py`
6. `/home/runner/work/ranger_ros2/ranger_ros2/ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/sim_messenger.py`
7. `/home/runner/work/ranger_ros2/ranger_ros2/ranger_base/src/ranger_messenger.cpp`

Only read URDF/mesh/world files when the task is explicitly about model visuals, Gazebo resources, or geometry.

## Default context exclusions
- `/home/runner/work/ranger_ros2/ranger_ros2/.git/**`
- `/home/runner/work/ranger_ros2/ranger_ros2/.claude_handoff/**`
- `/home/runner/work/ranger_ros2/ranger_ros2/docs/*.png`
- `/home/runner/work/ranger_ros2/ranger_ros2/**/meshes/*.dae`
- `/home/runner/work/ranger_ros2/ranger_ros2/**/meshes/*.{stl,obj,ply}`
- `/home/runner/work/ranger_ros2/ranger_ros2/**/*.rviz`
- `/home/runner/work/ranger_ros2/ranger_ros2/**/worlds/*.sdf`
- `/home/runner/work/ranger_ros2/ranger_ros2/**/build/**`
- `/home/runner/work/ranger_ros2/ranger_ros2/**/install/**`
- `/home/runner/work/ranger_ros2/ranger_ros2/**/log/**`
- `/home/runner/work/ranger_ros2/ranger_ros2/**/__pycache__/**`

## High-signal package ownership
- `ranger_base`: real robot driver interface and control behavior
- `ranger_mini_v3_sim`: Gazebo + ros2_control bringup
- `ranger_mini_v3_sim_messenger`: sim-side Twist and state parity node
- `ranger_msgs`: ROS interface contracts

## Deep history policy
`/home/runner/work/ranger_ros2/ranger_ros2/.claude_handoff/` is archival and can be large.  
Start with current code and package READMEs first; use handoff files only for historical debugging.
