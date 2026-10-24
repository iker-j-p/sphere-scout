# Requirements: detector verification and tests

## Introduction

`thermal_detector_node` asks an external HTTP model service about each thermal frame and publishes `/detected_target`. `model_client.py` converts the model's boundary-coordinate boxes to the repo's `pixel_convention`. Neither has unit tests, and the box convention and server contract rest on the model repo's documentation. This spec adds confidence without changing any interface.

## Requirements

### R1: Client contract is tested
1. `build_request` SHALL be tested for the happy path, wrong shape, and NaN/infinite input.
2. `parse_response` SHALL be tested for: no human, one human box, both `pixel_convention` values, missing keys, wrong types, non-finite numbers, and a response that is not a JSON object.
3. For `edge`, a box of boundary coordinates (left, top, right, bottom) SHALL give centre and size equal to (left + right) / 2 and (right - left). For `index`, the inclusive-index convention SHALL give right - 1 and bottom - 1 before conversion.
4. Tests SHALL run as plain scripts in `src/sphere_scout_ai/test/`, with no ROS graph and no server.

### R2: Node behaviour is verified
1. WHEN the model server is down THEN the node SHALL warn once and publish nothing on `/detected_target`.
2. WHEN the server answers "no human" THEN the node SHALL publish an empty array with the image header.
3. WHEN the server is slower than the frame rate THEN old frames SHALL be dropped and counted, and the executor SHALL not block.
4. Each published message SHALL carry the stamp and `frame_id` of its source image.

### R3: Model box convention is confirmed against the real server
1. WHEN a synthetic frame with a known hot region is sent to the running server THEN the returned box SHALL be checked against the documented right/bottom-exclusive convention.
2. IF the real behaviour differs from `model_client.py`'s header THEN the difference SHALL be reported and the client fixed in a separate, flagged change.

### R4: Pipeline check
1. WHEN the detector, localizer and viewer run together on the fake publisher or on the sim THEN the box overlay in `/thermal/view` SHALL sit on the warm blob.
2. `pixel_convention` SHALL be identical in `thermal_detector.yaml`, `target_localizer.yaml` and `thermal_viewer.yaml`.
