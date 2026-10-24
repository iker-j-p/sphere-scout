# Tasks: detector verification and tests

- [ ] 1. Unit tests for `model_client.py` (R1)
  - [ ] 1.1 Create `src/sphere_scout_ai/test/test_model_client.py` with request-building tests
  - [ ] 1.2 Add `parse_response` tests for both pixel conventions and all malformed-input cases
  - [ ] 1.3 Run it with the other three test scripts and add its command to `build-and-test` steering
  - _Requirements: R1_

- [ ] 2. Node behaviour against a stub server (R2)
  - [ ] 2.1 Add a scriptable stub HTTP server for tests
  - [ ] 2.2 Verify server-down, no-human, slow-server and header-copy behaviour
  - _Requirements: R2_

- [ ] 3. Check the real model's box convention (R3)
  - [ ] 3.1 Start the model with `tools/run_model_server.sh` and send a synthetic hot-region frame
  - [ ] 3.2 Compare with the convention in `model_client.py`; report any difference
  - _Requirements: R3_

- [ ] 4. Pipeline check (R4)
  - [ ] 4.1 Confirm `pixel_convention` matches across the three YAMLs
  - [ ] 4.2 Run detector + localizer + viewer and check the overlay position in `/thermal/view`
  - _Requirements: R4_
