# Tasks: operator view cleanup

- [ ] 1. Remove VR code from the web view (R1)
  - [ ] 1.1 Read `tools/xr_viewer/web/app.js` fully and list which symbols are XR-only
  - [ ] 1.2 Remove the XR import, button, renderer setup and session listeners
  - [ ] 1.3 Remove controller polling, snap turn, head-locked HUD and dollhouse code
  - [ ] 1.4 Simplify the animation loop to `controls.update()` and render
  - [ ] 1.5 Update the `bridge.py` docstring to drop Quest/headset wording
  - [ ] 1.6 Run `test_bridge.py` and, if Node is available, the three `.mjs` tests
  - _Requirements: R1_

- [ ] 2. Fix the dangling spec references (R2)
  - [ ] 2.1 Rewrite the first comment line in `target_localizer.yaml` and `thermal_viewer.yaml`
  - _Requirements: R2_

- [ ] 3. RViz layout (R3)
  - [ ] 3.1 User: save `sphere_scout.rviz` from the RViz2 GUI into `src/sphere_scout_description/rviz/`
  - [ ] 3.2 Remove the README placeholder row; confirm `operator.launch.py` loads the layout
  - _Requirements: R3_
