# ranger_ros2 sim port — running context

This file is the append-only running journal of the ROS 1 → ROS 2 Jazzy
port of the AgileX Ranger Mini v3 model. It exists so that a fresh
session (or a fresh teammate) can re-bootstrap by reading just this file
plus the matching `round_NN_*.md` handoff in this same directory.

## Workspace layout

- Workspace root:  `~/agilex_ws/`
- Repo:            `~/agilex_ws/src/ranger_ros2/` (this repo, multi-package)
- Sibling vendor:  `~/agilex_ws/src/ugv_sdk/` (do not modify)
- Target ROS:      ROS 2 Jazzy on Ubuntu 24.04
- Target sim:      Gazebo Harmonic (gz-sim 8.x)

## Phase plan

| Phase | Branch                       | Package                              | Status      |
|-------|------------------------------|--------------------------------------|-------------|
| 1     | `phase-1-description`        | `ranger_mini_v3_description`         | DONE (R01)  |
| 2     | `phase-2-ros2-control`       | `ranger_mini_v3_sim` (ros2_control)  | not started |
| 3     | `phase-3-gazebo-bringup`     | `ranger_mini_v3_sim` (Gazebo launch) | not started |
| 4     | `phase-4-messenger-node`     | `ranger_mini_v3_sim_bringup`         | not started |
| 5     | `phase-5-interface-parity`   | (real-driver topic parity)           | not started |

## Sacred constants (do not relitigate)

- Wheel radius `0.09 m`, wheelbase `0.50 m`, track `0.38 m`.
- Steering joint axis is `(0, 0, -1)` — keep this. Real driver assumes
  positive command = CW from above.
- Asymmetric `rr_steering_joint` origin from the original ROS 1 model
  is fixed to `(-0.25, -0.19, -0.1)`. Do not regress.
- Real-driver topic interface that sim must match:
  - sub: `/cmd_vel`           (geometry_msgs/Twist)
  - pub: `/odom`              (nav_msgs/Odometry)
        `/system_state`       (ranger_msgs/SystemState)
        `/motion_state`       (ranger_msgs/MotionState)
        `/actuator_state`     (ranger_msgs/ActuatorStateArray)
        `/battery_state`      (sensor_msgs/BatteryState)
  - tf: `odom -> base_link` (gated by `publish_odom_tf`)
- Mode switching (sim must mirror real driver):
  lateral-y dominant → parallel; pure ω → spinning; otherwise dual-Ackermann.

## Hard non-goals

Do NOT modify (without an explicit prompt instruction): `ranger_base/`,
`ranger_bringup/`, `ranger_msgs/`, `ugv_sdk/`, anything outside
`~/agilex_ws/src/ranger_ros2/`. We DEPEND on `ranger_msgs` but never
edit it.

---

## Round log

### 2026-05-11 — Round 01 — Phase 1 description package import

- **Branch:** `phase-1-description` (created from `humble`; see deviation
  note below — the bootstrap nominally specifies `main`, but no `main`
  branch exists in the pre-existing repo).
- **Package added:** `ranger_mini_v3_description/` — pure URDF/xacro +
  meshes, no Gazebo / ros2_control dependencies. Reusable by
  nav2/MoveIt/RViz.
- **Commits on branch:**
  - `a876c9b` `feat(sim): add ranger_mini_v3_description package (phase 1)`
  - + `docs(handoff): round 01 description package import`
- **Files created (this round):**
  - `ranger_mini_v3_description/{package.xml, CMakeLists.txt, README.md}`
  - `ranger_mini_v3_description/urdf/ranger_mini_v3.xacro` (161 lines)
  - `ranger_mini_v3_description/launch/display.launch.py`
  - `ranger_mini_v3_description/rviz/display.rviz`
  - `ranger_mini_v3_description/meshes/{ranger_base,steering_wheel,wheel_v3}.dae`
    (binary, ≈37.8 MiB total)
  - `ranger_mini_v3_description/config/` (empty, **not** in git — see
    open question 2 in round_01 handoff)
  - `.claude_handoff/{BOOTSTRAP_ACK.md, round_01_description.md, CONTEXT.md}`
- **Verification done:** xacro parses (275-line URDF, exit 0); colcon
  builds the package alone in ~1.2 s; `ros2 pkg prefix` finds it under
  `install/`; `ros2 launch ... --print` produces a valid launch
  description. RViz visual verification deferred to the operator (see
  "Manual launch steps for the operator" in `round_01_description.md`).
- **Deviations:** branched from `humble`, not `main` (no `main` exists);
  empty `config/` dir from the zip is not git-tracked; commit author
  comes from the host's git config (`solanrewaju2020@fau.edu`),
  unrelated to the placeholder maintainer in `package.xml`.
- **Open questions for architect:** (1) base-branch policy — keep
  `humble`, switch to `jazzy`, or rename to `main`? (2) what to do
  about the empty `config/` dir vs. the `install(DIRECTORY ... config
  ...)` line in CMakeLists; (3) update `package.xml` maintainer field
  or leave as a per-deployer placeholder? Full detail in
  `round_01_description.md`.
