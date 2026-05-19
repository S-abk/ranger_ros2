# Copilot instructions for ranger_ros2

## Session defaults
- Start with `docs/onboarding_index.md`
- Keep context focused on the task package(s) only.
- Treat `.claude_handoff/` as archival; if needed, use only `.claude_handoff/CONTEXT.md`.

## Prompt-bloat controls
- Do not ingest mesh/image/world files unless the task needs geometry or visuals.
- Prefer launch entrypoints and message definitions over historical notes.
- For sim behavior, prioritize:
  - `ranger_mini_v3_sim_messenger/.../sim_messenger.py`
  - `ranger_base/src/ranger_messenger.cpp`

## Safety boundaries
- Do not modify sibling vendor repositories (for example `ugv_sdk` in workspace `src/`).
- Keep ROS topic/message compatibility with `ranger_msgs` contracts.
