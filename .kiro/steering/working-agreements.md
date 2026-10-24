---
inclusion: always
---

# Working agreements

General practices for anyone (human or agent) changing this repo. They add to the hard rules in `AGENTS.md`, they don't replace them.

## Scope and decisions
- Read the file docstring/header before editing. It holds the spec for that file. Implement what it says, nothing beyond it.
- If the spec is ambiguous or a value is missing, ask. Never invent a physical value, a pose, a pin number, or a convention. Leave the `TODO(kickoff)` marker and list it in `README.md`.
- Keep a change as small as the task allows. No drive-by refactors, renames or reformatting in the same diff.
- Do not rewrite tested reference code (`triangulation.py`, `thermal_render.py`). Extend around it, and ask first if a change is unavoidable.
- When something fails twice the same way, stop patching. State the root cause, then propose a different approach.

## Interfaces
- A topic, message type, frame, parameter name, bridge entry or the thermal encoding is shared with other packages and the firmware. Changing any of them is an interface change: say so explicitly, and update `AGENTS.md`, `README.md`, `bridge.yaml` and the firmware header comment in the same change. The `#interface-change` steering file has the checklist.
- Standard message types only. No custom interface package.

## Verification
- After a code change, run the checks that cover it (see `build-and-test.md`) and read the result. A command that exits 0 is not proof the behaviour is right.
- If something cannot be verified here (no Gazebo run, no headset, no hardware, Node or PlatformIO not installed), say so plainly instead of implying it passed.
- For anything geometric (frames, signs, axes), check a known case by hand, such as a target dead ahead, before trusting the code.

## Safe operation
- Never `pkill` or `killall` by process name. The user may be running their own demo. Start test processes with their own `ROS_DOMAIN_ID` (e.g. 87 or 88) and their own port, and stop them by PID.
- Two publishers of `/clock` or TF on one domain corrupt each other (symptom: floods of `TF_OLD_DATA`). Isolate with a domain ID.
- Do not commit, push, or create branches unless asked. Stage files by name, never `git add -A`.
- Servers started by tools here (`bridge.py`, the model server) have no authentication. Keep them on `127.0.0.1` unless the network is trusted, and flag it when a change widens exposure.
- Do not read or print the contents of `tools/xr_viewer/.cert/*`. It is git-ignored and must stay so.

## Code style
- Python: match the surrounding style. Docstring at the top of each node states what it subscribes to, publishes, and its parameters; update it when those change.
- Keep ROS-free logic in pure modules (no `rclpy` imports) so it can be tested without a ROS graph.
- Nodes must not block the executor: slow I/O goes to a worker thread with a one-slot mailbox, as in `thermal_detector_node`.
- Prefer failing loudly and visibly (log a warning, show a banner) over silently publishing a wrong value. A dead dependency must never look like "nothing seen".

## Communication
- Short answers. Say what changed and what was verified. Do not recap every file.
- Documents and comments describe the current state. Don't record history ("previously", "no longer") in code comments.
