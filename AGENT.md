# Agent onboarding guide (sim_jazzy)

This repository contains both runtime code and large simulation assets.  
To avoid context bloat, follow this order and scope.

## Read-first order (minimal context)
1. `README.md`
2. `ranger_mini_v3_sim/README.md`
3. `ranger_msgs/msg/*.msg`
4. `ranger_mini_v3_sim/launch/gazebo_full.launch.py`
5. `ranger_mini_v3_sim_messenger/launch/messenger.launch.py`
6. `ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/sim_messenger.py`
7. `ranger_base/src/ranger_messenger.cpp`

Only read URDF/mesh/world files when the task is explicitly about model visuals, Gazebo resources, or geometry.

## Default context exclusions
- `.git/**`
- `docs/*.png`
- `**/meshes/*.dae`
- `**/meshes/*.{stl,obj,ply}`
- `**/*.rviz`
- `**/worlds/*.sdf`
- `**/build/**`
- `**/install/**`
- `**/log/**`
- `**/__pycache__/**`

## High-signal package ownership
- `ranger_base`: real robot driver interface and control behavior
- `ranger_mini_v3_sim`: Gazebo + ros2_control bringup
- `ranger_mini_v3_sim_messenger`: sim-side Twist and state parity node
- `ranger_msgs`: ROS interface contracts

## Deep history policy
Start with current code and package READMEs first.  
Use git history and pull requests for historical debugging context.
