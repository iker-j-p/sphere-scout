---
inclusion: manual
---

# Interface change checklist

Use with `#interface-change` when a change touches a topic name, message type, frame name, parameter name, bridge entry, service, or the thermal encoding.

1. State in the reply that this is an interface change and list what changed.
2. Update the topics table in `AGENTS.md` and `README.md` in the same change. Keep them identical.
3. Update `src/sphere_scout_bringup/config/bridge.yaml` if the topic crosses Gazebo. Gazebo-side names differ only for `/shell/cmd` (gz `/shell_cmd`).
4. Update the YAML that carries the parameter: `target_localizer.yaml`, `thermal_viewer.yaml`, `thermal_detector.yaml`. `pixel_convention` must match in all three.
5. Update the docstring at the top of every node that publishes or subscribes to it.
6. Update the firmware header comment in `firmware/sphere_scout_firmware/src/main.cpp` if the firmware publishes or subscribes to it.
7. Check consumers: `tools/xr_viewer/bridge.py`, RViz layouts, `tools/record_thermal.py`, `fake_detection_pub.py`.
8. Re-run the unit tests and the no-Gazebo fixture (`build-and-test.md`). Note what could not be run.
9. Do not add a custom interface package. Use standard message types.
