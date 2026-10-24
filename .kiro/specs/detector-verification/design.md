# Design: detector verification and tests

## Approach

Test the pure client first (no ROS, no server), then exercise the node against a stub HTTP server, then check the real model once.

## Components

- `src/sphere_scout_ai/test/test_model_client.py`: plain script in the repo's existing style (prints `ok <name>`). Imports `sphere_scout_ai.model_client` directly. Builds response dictionaries by hand.
- Stub server (test-only, in the same test file or a small helper): `http.server` on `127.0.0.1` with an ephemeral port, scripted to return a fixed JSON reply, delay, or an error. Used for R2 against `ModelClient.predict`.
- Node-level check (R2.3, R4): run `thermal_detector_node` under a private `ROS_DOMAIN_ID` with the fake publisher providing `/clock` and images, and the stub (or real) server on a private port. Observe `/detected_target` with `ros2 topic echo` and the stats log line every 30 s.
- Real-server check (R3): a short script that sends one synthetic frame to `tools/run_model_server.sh`'s server and prints the box.

## Notes

- The model server and the browser bridge must use different ports (8766 vs 8765).
- A real frame needs the model's checkpoint in `external/thermal_rescue_ai_model/models/`. If it is missing, report it rather than faking a model reply.
- No interface changes. If R3 shows the convention differs, fixing `parse_response` is a behaviour change to flag in the report, not a rewrite of the reference modules.
