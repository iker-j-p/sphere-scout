---
inclusion: fileMatch
fileMatchPattern: ["tools/xr_viewer/**", "src/sphere_scout_ai/**/target_localizer_node.py", "src/sphere_scout_ai/**/thermal_viewer_node.py"]
---

# Operator side: localizer, viewer, browser bridge

- `tools/xr_viewer/` is read-only and external (`COLCON_IGNORE`d). It subscribes to `/survivor/markers`, `/thermal/view` and TF `odom -> base_link`. It must not add a topic, type or frame, and must not publish.
- Default bind is `127.0.0.1:8765`. `--host 0.0.0.0` has no authentication; flag any change that makes it reachable from a network by default.
- Thermal frames are encoded only while a browser is connected and are never replayed to a new client, so an old frame cannot pass for a live one. Keep that.
- `spawn_pose` for the viewer is read from `target_localizer.yaml`, the single source of truth. Do not duplicate it.
- The vendored three.js lives in `web/vendor/three/build/`. `.gitignore` has an exception for it; keep it.
- Web code is plain ES modules, no bundler. Keep `frames.js`, `markers.js`, `panel.js` free of DOM or three.js side effects at import time so the Node tests can import them.
- The headset gets a separate application. Don't add VR/XR features to this web view.
- Localizer rules: deterministic function of detections and TF. Statuses are descriptive (NO_TARGET, BEARING_ONLY, PROVISIONAL, LOW_CONFIDENCE, CONFIRMED), never advice. `/survivor/position` carries triangulated points only (not PROVISIONAL).
- Verify the geometry after any change with the no-Gazebo fixture (see `build-and-test.md`): sideways drive reaches CONFIRMED near the fixture truth, parked stays BEARING_ONLY, and `/survivor/position` stays silent when parked.
