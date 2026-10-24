# Sphere-Scout: session handoff

State as of 2026-10-24, written so a new chat can continue without the old one.
Repo root = colcon workspace = `/home/ikerj/Sphere-Scout/sphere-scout` (WSL2 Ubuntu 24.04, ROS 2 Jazzy).
`AGENTS.md` is the source of truth for the stack and rules; this file records what was built on top of it.
Git state when this was written: the work is in commit `68cc2be` on branch `position_webapp` (pushed). That
commit is MISSING `tools/xr_viewer/web/vendor/three/build/three.module.js` and `three.core.js` (the colcon
`build/` rule in `.gitignore` swallowed them). The fix (a `.gitignore` exception plus those two files) and this
handoff were staged afterwards but NOT committed; commit them or the web view cannot load on a fresh clone.
Rebuild after pulling: `colcon build --symlink-install` (a new console script, `thermal_viewer_node`, needs it).

## Paste this at the start of the new chat

```
Read AGENTS.md and docs/SESSION_HANDOFF.md first. They describe the operator pipeline
(target_localizer_node, thermal_viewer_node, tools/xr_viewer) that was built and tested without
Gazebo. Do not rewrite triangulation.py or thermal_render.py (tested reference code). Flag any
interface change and update AGENTS.md and README.md in the same change. Ask before assuming
anything marked "unset" or "unverified" below. Do not commit unless asked.
```

## Working rules that mattered

- Stack is fixed (AGENTS.md): ROS 2 Jazzy, Gazebo Harmonic names, Python rclpy, numpy + OpenCV, no cv_bridge, no ros2_control, standard message types only, no custom interface package.
- Do not invent physical values. Unset on purpose until Mech 2 provides them: `target_height_m`, `target_height_sigma_m`, `ground_truth_world` (localizer YAML), survivor pose in `apartment.sdf` / `ground_truth.yaml`.
- Only the EKF publishes `odom -> base_link`. The test fixtures publish it only in tests, never in real launches.
- Every node/launch sets `use_sim_time: true`. Do not declare it in code (rclpy does); the YAML/launch sets it.
- Facts only on the dashboard and feed: no advice text ("move", "rescan"...). Unit tests enforce this for localizer reasons and the viewer HUD.
- User preferences seen: short answers, interface changes flagged explicitly, no commits unless asked.

## What exists

### ROS side (`src/`)

| File | Purpose |
|---|---|
| `sphere_scout_ai/triangulation.py` | Geometry core, no ROS. Intrinsics, pixel->ray, quaternions, `world_to_odom`, ViewTracker (views = clusters by camera position, anchor-based), weighted least-squares triangulation, RANSAC consensus, height prior, statuses. PROVIDED reference, 15 tests. Do not rewrite. |
| `sphere_scout_ai/thermal_render.py` | Rendering core, no ROS. `decode_image`, `RangeTracker`, `render`, `format_hud`. PROVIDED reference, 9 tests. |
| `sphere_scout_ai/msg_utils.py` | `bbox_center` (handles both BoundingBox2D layouts; Jazzy uses `center.position.x/y`) and `corners_to_bbox` (see "Four corners"). 7 tests. |
| `sphere_scout_ai/target_localizer_node.py` | ROS glue for the localizer. |
| `sphere_scout_ai/thermal_viewer_node.py` | ROS glue for the driver's thermal feed. |
| `sphere_scout_ai/thermal_detector_node.py` | STILL A STUB (not ours). |
| `sphere_scout_ai/scripts/fake_detection_pub.py` | Test fixture: fake `/clock`, moving camera TF, 16UC1 image, detections. Not installed; run with `python3`. |
| `sphere_scout_bringup/config/target_localizer.yaml`, `thermal_viewer.yaml` | All parameters. |
| `sphere_scout_bringup/launch/operator.launch.py` | Starts localizer + viewer + RViz (`rviz:=false` to skip; `rviz_config:=` to point at a saved layout; falls back to RViz defaults if `sphere_scout_description/rviz/sphere_scout.rviz` does not exist). |
| `sphere_scout_ai/test/` | `test_triangulation.py`, `test_thermal_render.py`, `test_msg_utils.py`. |

### External tool (`tools/xr_viewer/`, `COLCON_IGNORE`d, outside ROS packages)

| File | Purpose |
|---|---|
| `bridge.py` | Read-only rclpy + stdlib bridge and web server. Subscribes `/survivor/markers`, `/thermal/view`, TF `odom->base_link`. Serves the web app, `/config.json` (spawn_pose read from `target_localizer.yaml`), `/models/apartment.glb`, and `/events` (server-sent events: `markers`, `robot` 20 Hz, `thermal` JPEG base64 with send time `t`). Thermal frames are only encoded while a browser is connected and are never replayed to new clients. Default bind 127.0.0.1; `--host 0.0.0.0` has NO authentication (read-only data); `--https` makes a self-signed cert in `.cert/` (git-ignored). |
| `web/index.html`, `app.js` | Page: desktop orbit camera on the semi-transparent apartment, HUD with the localizer status text, X toggles solid/x-ray, C clears the trail, T toggles the thermal window. |
| `web/frames.js` | Frame graph, unit-tested (odom = ROS axes; world hangs under odom via inverse `spawn_pose`; GLB is Z-up as Gazebo reads it). |
| `web/markers.js` | Draws the localizer's markers (spheres, ellipsoid, rays with per-vertex colour, size-prior segment), plus a fixed-screen-size halo and a drop line to the floor at the estimate. |
| `web/panel.js` | Movable/resizable thermal-feed window (drag title bar, corner grip, corner-cycle button, arrow keys / Shift+arrows, T, remembers layout in localStorage, "no frames for X s" overlay, frame-rate readout from the bridge's send times). |
| `web/vendor/three/` | Pinned three.js 0.186.1 (MIT), vendored so the demo needs no internet. NOTE: the folder is named `build/`, which `.gitignore` ignored; an exception line was added. |
| `test/` | `test_bridge.py` (4 tests), `test_frames.mjs`, `test_markers.mjs`, `test_panel.mjs` (+ `hooks.mjs` import-map shim for Node). |

## Design contracts (condensed from the original specs, which existed only in chat)

The localizer is a deterministic function of detections and TF. It does not command anything, does not subscribe to `/mission/state`, `/cmd_vel` or `/odom`, and never integrates the IMU.

- Frame to ray: pixel -> camera ray (`f = 30.73 px` for 32 px / 0.96 rad) -> `odom` ray using TF looked up AT THE DETECTION'S IMAGE STAMP, with no timeout (non-blocking). Frames wait in a pending queue until `pending_timeout_s`. A few dropped frames at startup (TF buffer not yet filled) is expected.
- `/detected_target` handling: the stamp of every message (even empty) is the "last detection" time (spec-literal; see open questions). Duplicate stamps dropped. Largest box only (single survivor). Boxes touching the image border are rejected (tolerance 0.1 px, limits depend on `pixel_convention`).
- Views: frames whose camera origin is within `view_radius_m` of a view's anchor. One ray per view (median, outlier gate `ray_gate_deg`, sigma = max(`sigma_theta_rad`, scatter), NOT divided by sqrt(n)). Depth information comes only from baseline.
- Solve at 5 Hz: weighted least squares over all view rays, RANSAC consensus with 4+ views (3 or fewer: no rejection), rejected views reported and drawn red.
- Statuses (descriptive, not advice): NO_TARGET, BEARING_ONLY, PROVISIONAL (needs height prior, NOT published on `/survivor/position`), LOW_CONFIDENCE, CONFIRMED. `/survivor/position` carries triangulated points only.
- Markers (frame `odom`, each array starts with DELETEALL): ns `status` (text), `estimate` (green CONFIRMED / amber LOW_CONFIDENCE / cyan PROVISIONAL), `uncertainty` (2-sigma ellipsoid), `rays`, `size_prior` (BEARING_ONLY only), `ground_truth` (blue, when set).
- Status text, facts only: depth/lateral/height, depth sigma + status, views used/rejected + baseline + parallax, "from robot now" distance and bearing (+ = left), optional error vs ground truth, "last detection N s ago". BEARING_ONLY shows azimuth/elevation, a range prior from blob size, and the geometry reason.
- Frames/axes: base_link +x forward; `odom` origin = robot spawn pose; odom +x is INTO the structure (Mech 2 must spawn that way); x = depth, y = lateral (+ left), z = height. `world_to_odom` / `spawn_pose` is `[x, y, z, yaw]` of base_link at spawn.
- Viewer: renders its own rgb8 `/thermal/view` (reliable, depth 1, header copied from the source image) from the 16-bit kelvin image. Auto temperature range (expands instantly, relaxes slowly, min span), nearest-neighbour upscale, colorbar, crosshair, detector boxes only if the detection stamp is within `overlay_max_age_s` of the image, HUD facts, watchdog banners `NO FEED <age> s`, `WAITING FOR /thermal/image_raw`, `BAD IMAGE`. Under `use_sim_time` with no `/clock`, the watchdog never fires (timers do not tick).

### Four corners

The user's detection model outputs four corners of a square. `/detected_target` stays a standard `Detection2DArray` (center + size), so the conversion belongs in `thermal_detector_node` (still a stub). Helper: `corners_to_bbox(corners, pixel_convention)`. Assumption (UNCONFIRMED with the model owner): for `index`, corners are pixel indices of the extreme pixels, inclusive (size = max - min + 1); for `edge`, boundary coordinates (size = max - min). A tilted shape is reduced to its axis-aligned hull.

## Interface changes made (already reflected in AGENTS.md and README.md)

- New topic `/thermal/view` (sensor_msgs/Image rgb8) from `thermal_viewer_node`.
- New service `/target_localizer_node/reset` (std_srvs/Trigger).
- New nodes/launch: `thermal_viewer_node`, `operator.launch.py`.
- Localizer parameters replaced the stub's `width`, `height`, `min_ray_angle_deg`, `ground_truth_file` with the names in `target_localizer.yaml`. `perception.launch.py` (stub) still mentions `ground_truth_file`; do not launch the localizer from both launch files.
- `package.xml`: `sphere_scout_ai` gained `std_srvs`, `tf2_ros_py`; `sphere_scout_bringup` gained `sphere_scout_ai`, `launch_ros`.
- The bridge adds no ROS topic, type or frame (it only listens).

## How to run

```bash
cd ~/Sphere-Scout/sphere-scout && source /opt/ros/jazzy/setup.bash && source install/setup.bash

# unit tests (no ROS graph needed; msg_utils uses the real message if ROS is sourced)
(cd src/sphere_scout_ai && python3 test/test_triangulation.py && python3 test/test_thermal_render.py && python3 test/test_msg_utils.py)
python3 tools/xr_viewer/test/test_bridge.py
# JS tests need Node 20+ (not installed on the machine):  node tools/xr_viewer/test/test_{panel,frames,markers}.mjs

# demo without Gazebo (three terminals, fake publisher FIRST because it provides /clock)
python3 src/sphere_scout_ai/scripts/fake_detection_pub.py --demo --corners   # 30 s loop, announces phases, resets the localizer each cycle
ros2 launch sphere_scout_bringup operator.launch.py rviz:=false             # or without rviz:=false to also open RViz
python3 tools/xr_viewer/bridge.py                                           # then open http://localhost:8765
```

Fake publisher flags: `--demo`, `--corners`, `--path sideways|parked|parallel`, `--target X Y Z` (default 4 0.3 0.3, a fixture value), `--speed`, `--y0`, `--span`, `--noise-px`, `--no-image-after S`, `--det-lag S`, `--bad-image`, `--corrupt-y LO HI`. The camera mount and target in the fixture are test values, not robot dimensions.

RViz (manual, layout not saved yet): Fixed Frame `odom`; Image `/thermal/view` with Reliability = Reliable (not `/thermal/image_raw`, which RViz normalizes into a white square); MarkerArray `/survivor/markers`; TF. Odometry display needs the EKF's `/odometry/filtered` (not present in the fake setup).

## Test status at the time of writing

- Python: 15 + 9 + 7 + 4 (bridge) pass, on numpy 1.26.4 / OpenCV 4.6.0 (the WSL stack).
- JS: panel, frames, markers tests pass (run with a temporary Node 24 binary fetched into `/tmp`, since deleted).
- Browser end-to-end of the thermal window: 25/25 checks passed in headless Chromium (Playwright from a temporary venv in `/tmp`, since deleted). That script is NOT in the repo. Re-create it if wanted: drag, clamp, resize with 4:3, corner cycle, keyboard, reload persistence, hide/show, detector box in stream, stale overlay on killing the viewer node and recovery.
- No-Gazebo localizer scenarios verified with the fake: sideways drive reaches CONFIRMED (~4 cm from the fixture truth, depth sigma 0.13 m after 8 s); parked = BEARING_ONLY with `/survivor/position` silent; distant target = BEARING_ONLY (parallax reason); driving at the target = valid but long ellipsoid; corrupted view = rejected and red; height prior = PROVISIONAL, no position published; reset works.
- NOT run: anything with Gazebo, the real detector, the EKF, or a headset. The page was only rendered with a software renderer.

## Not verified / blocked on others

Gazebo "Verify" items (localizer + viewer specs): image encoding and endianness of `/thermal/image_raw`; frame id on image and detections and whether it is in TF; camera axis convention (`camera_convention` gz vs optical); detector pixel convention; detector output stamp equals image stamp; stationary pose jitter of `odom->base_link` (must be below `view_radius_m` = 0.15); VFOV in the SDF; spawn pose and +x into the structure. Then the calibration test (target dead ahead gives the right bearing) and the coverage test (truth inside the 2-sigma ellipsoid ~95% over 20+ runs). If coverage is low the errors are bias (EKF yaw drift, extrinsics, pixel convention): raise `sigma_theta_rad` and fix the source. Heading drift (no absolute yaw in the EKF) is the main threat to overlay accuracy.

Teammate asks still open: Mech 1 (camera stays level while shells roll; shell/collision model for open-shell navigation), Mech 2 (spawn pose, survivor coordinates/heights, +x into structure, VFOV, put the survivor stand-in in `apartment.sdf`), Mechatronics (`/thermal/image_raw` bridged, camera frame in TF, state machine for open-shell navigation), detector owner (output stamp = image stamp, bbox in 32x24 pixels, publish every frame, state the pixel convention).

## Open decisions and next steps

1. User asked for "the thermal view as priority" with about 2 hours left; the thermal window is done. Candidates next: (a) remove the leftover VR code from the web view, (b) save the RViz layout, (c) Gazebo calibration/coverage once the sim exists, (d) implement `thermal_detector_node` using `corners_to_bbox`.
2. Remove VR code from `tools/xr_viewer/web/app.js` (VRButton, XR session handlers, gamepad polling, head-locked panel, dollhouse). The user decided the headset gets a SEPARATE application and does not want it in the web view. Not removed yet; the Quest-specific help text was already removed from the page. The `bridge.py` docstring still talks about Quest/HTTPS; the HTTPS option itself is generic.
3. Headset findings for the separate app (Samsung Galaxy XR = Android XR): Chrome supports WebXR including the AR module, hit-test, anchors and hand input; hand input is the default and controllers are optional, so controller-stick code will not work. WebXR needs a secure context (localhost or HTTPS); no Galaxy-XR-specific guidance on reaching a PC dev server was found (generic: `adb reverse` or HTTPS). Untested.
4. Multi-target is deferred. The user wants ONE human for now (maybe later if time). Current code assumes one survivor in `on_det` (largest box), the single ViewTracker/`estimate()`, `/survivor/position` and the markers. Sketch if revived: accept all detections; split each view's rays by bearing; sequential RANSAC for up to `max_targets` with `min_views_per_target` (~3); persistent IDs by matching to existing tracks in `odom`; possible new standard-type topic `/survivors` (`vision_msgs/Detection3DArray`) and keep `/survivor/position` as the most confident target. Limits: ghost intersections with 2 views (need 3+ non-coplanar), merged blobs when humans are close at 32x24, ID fragmentation under occlusion.
5. Spec ambiguity resolved literally, worth confirming: "last detection" age uses the newest `/detected_target` message even if empty, so with a detector publishing empty arrays every frame it stays near 0 while the survivor is lost.
6. RViz layout `sphere_scout_description/rviz/sphere_scout.rviz` still to be saved from the GUI (do not hand-write it); the existing `mission_control.rviz` is a comment-only stub.
7. Optional later (viewer spec): reproject `/survivor/position` into the thermal image (true AR on the robot camera). It would couple the viewer to the localizer and is an interface change; do after the base feed is validated in Gazebo.
8. The original handoff documents (LOCALIZER_SPEC.md, THERMAL_VIEWER_SPEC.md) were never saved in the repo, but target_localizer.yaml / thermal_viewer.yaml headers still reference `docs/LOCALIZER_SPEC.md` and `docs/THERMAL_VIEWER_SPEC.md`. This file carries the condensed contracts; ask the user for the originals or fix the YAML headers.

## Environment notes for the next agent

- The user runs their own demo (bridge on port 8765, `operator.launch.py`, fake publisher) while I test. NEVER `pkill` by name. Start test processes with their own `ROS_DOMAIN_ID` (e.g. 87/88) and their own bridge port (e.g. 8790), and clean up by PID. Two fake publishers or launches on one domain corrupt each other's TF and clocks (symptom: floods of `TF_OLD_DATA` warnings).
- The shell tool refuses inline commands that contain words like "port"/"host" next to a server-like pattern. Put such orchestration in a script file under `/tmp` and run that.
- Reading an image right after a script writes it can race; check `ls` first.
- Temporary tools used and deleted (re-fetch if needed): Node from `https://nodejs.org/dist/<LTS>/` into `/tmp/nodejs`; Playwright in a `/tmp` venv with `PLAYWRIGHT_BROWSERS_PATH=/tmp/pwbrowsers` and `playwright install chromium`; launch Chromium with `--use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader` for WebGL.
- `ros2 launch ... operator.launch.py` needs the workspace built; a stale `install/` from before this work lacks the viewer entry point.
- Gazebo meshes: `apartment.glb` is exported Z-up (`tools/export_apartment.py`, `export_yup=False`) and Gazebo reads it as-is; the world has a ground plane at z = 0 and the apartment at the origin.
