# Design: operator view cleanup

## R1: VR removal in `app.js` (290 lines)

Remove, in this order, checking the file still loads after each step:

1. `import { VRButton ... }` and `document.body.appendChild(VRButton.createButton(renderer))`.
2. `renderer.xr.enabled`, `renderer.xr.setReferenceSpaceType`, and the `sessionstart` / `sessionend` listeners.
3. Controller code: `pressed`, `turnRig`, `pollControllers`, `snapReady`, `prevButtons`, and the temporaries only they use (`tmpHead`, `tmpDir`, `tmpRight`, `UP`).
4. Head-locked HUD mesh (`hudMesh`, `drawHud`, `hudDirty`) if it is only used in XR; keep the DOM HUD that desktop uses.
5. Dollhouse (`DOLLHOUSE_POS`, `setDollhouse`, the `rig`/`viewRoot` offset) if only reachable in XR. If `rig` also holds the camera on desktop, keep a minimal group and delete only the XR transforms.
6. Main loop: `renderer.setAnimationLoop` calls `controls.update()` and `renderer.render`.

Read the code before each deletion: shared helpers must stay. Do not touch `frames.js`, `markers.js` or `panel.js`.

The `vendor/three/addons/webxr/VRButton.js` file can stay (vendored, unused); removing vendored files is optional.

## R2: YAML headers

Replace the first comment line in each YAML with a self-contained summary (one or two lines of what the file configures and where `(A)` tuning notes apply). Do not rename anything. A single new `docs/` file would be a design decision about documentation, so only create one if the user asks.

## R3: RViz layout

This needs the GUI. The agent prepares nothing but the instruction; the user saves the file from RViz2 into `src/sphere_scout_description/rviz/sphere_scout.rviz` and rebuilds if the install layout is not symlinked. Then update the README placeholder table.

## Verification

- `python3 tools/xr_viewer/test/test_bridge.py`.
- JS tests need Node 20+ (`test_frames.mjs`, `test_markers.mjs`, `test_panel.mjs`), not installed on the dev machine.
- Manual: start the fake publisher, the operator launch and `bridge.py`, then open `http://localhost:8765` and check orbit, X, C, T and the thermal window. Not verifiable by the agent without a browser.
