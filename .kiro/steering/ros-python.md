---
inclusion: fileMatch
fileMatchPattern: ["src/**/*.py", "tools/**/*.py"]
---

# ROS 2 Python conventions

- rclpy only. No C++ nodes, no ros2_control, no custom interface package.
- `use_sim_time: true` comes from the YAML or the launch file. Do not declare `use_sim_time` in node code (rclpy declares it).
- Parameters: declare every parameter with a default, read it once in `__init__`, and document it in the file docstring. Parameter names are shared interface (see `#interface-change`).
- Keep geometry and image maths in ROS-free modules. `triangulation.py` and `thermal_render.py` are tested reference code: do not rewrite them. Add behaviour around them.
- `cv_bridge` is not used. Decode `sensor_msgs/Image` with `thermal_render.decode_image`, which handles `mono16`/`16UC1`, endianness and row padding.
- Thermal frames are 32 x 24, 16-bit, kelvin x 100. Row 0 is the top. Boxes are in pixels of that image, in the configured `pixel_convention`.
- TF lookups use the image stamp, not "latest", and do not block (no timeout). Frames wait in a pending queue.
- Sensor subscriptions use `qos_profile_sensor_data`. `/thermal/view` is reliable with depth 1 so RViz shows it.
- Never block the executor with network I/O. Hand the work to a thread through a one-slot mailbox (see `thermal_detector_node`) and count dropped frames.
- Publish nothing when the input is unusable. Do not publish an empty `Detection2DArray` for a frame the model did not answer: an empty array means "no human".
- A new console script needs an entry in `setup.py` and a rebuild.
- Tests are plain scripts printing `ok <name>` in `src/sphere_scout_ai/test/`. Keep new pure logic testable the same way.
- Frame ids and topic names are shared across packages. Check `AGENTS.md` before adding either.
