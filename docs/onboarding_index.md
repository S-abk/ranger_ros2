# Onboarding index (agent-optimized)

Use this checklist to keep onboarding focused and fast.

1. Read `README.md`
2. Read `ranger_mini_v3_sim/README.md`
3. Read message contracts in `ranger_msgs/msg/`
4. Read launch entrypoints:
   - `ranger_mini_v3_sim/launch/gazebo_full.launch.py`
   - `ranger_mini_v3_sim/launch/gazebo.launch.py`
   - `ranger_mini_v3_sim_messenger/launch/messenger.launch.py`
5. Read sim runtime:
   - `ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/sim_messenger.py`
6. Read real-driver parity reference:
   - `ranger_base/src/ranger_messenger.cpp`
7. Read URDF/xacro only if needed:
   - `ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro`
   - `ranger_mini_v3_description/urdf/ranger_mini_v3.xacro`
8. Avoid by default:
   - `.claude_handoff/`
   - `ranger_mini_v3_description/meshes/`
   - `docs/*.png`
