# Requirements: operator view cleanup

## Introduction

Three loose ends on the operator side: leftover VR code in the browser view (the headset will be a separate application), config headers pointing at spec documents that are not in the repo, and the RViz layout that is still unsaved. No ROS interface changes.

## Requirements

### R1: Remove VR code from the web view
1. `tools/xr_viewer/web/app.js` SHALL no longer import or use `VRButton`, XR session handlers, gamepad polling, the head-locked HUD panel, or the dollhouse mode.
2. `renderer.xr` setup SHALL be removed and the animation loop SHALL only update `OrbitControls` and render.
3. Desktop behaviour SHALL be unchanged: orbit camera, X solid/x-ray, C clear trail, T thermal window, thermal feed, markers, HUD status text.
4. The `bridge.py` docstring SHALL stop describing Quest/headset use. The `--https` and `--host` options SHALL stay (they are generic) and keep their no-authentication warning.
5. Existing tests for `frames.js`, `markers.js`, `panel.js` and `test_bridge.py` SHALL still pass. IF Node is not installed THEN this SHALL be stated, not skipped silently.

### R2: Fix dangling document references
1. `target_localizer.yaml` and `thermal_viewer.yaml` headers reference `docs/LOCALIZER_SPEC.md` and `docs/THERMAL_VIEWER_SPEC.md`, which do not exist.
2. The headers SHALL either point at an existing document or state the contract in a short comment. The "(A)" assumption markers in the localizer YAML SHALL stay meaningful.
3. No parameter name or value SHALL change.

### R3: Save the RViz layout
1. The layout `sphere_scout_description/rviz/sphere_scout.rviz` SHALL be saved from the RViz2 GUI by the user (not hand-written), with: Fixed Frame `odom`; Image `/thermal/view` with Reliability = Reliable; MarkerArray `/survivor/markers`; TF.
2. `operator.launch.py` SHALL find it by default (it already falls back to RViz defaults if missing).
3. The README `TODO(kickoff)` row for the layout SHALL be removed once the file exists.
