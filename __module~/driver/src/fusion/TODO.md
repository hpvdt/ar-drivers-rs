
## Tasks

- [ ] Return calibration values in caller-visible units
    - Samples are divided by `pre_scaler` before storage, but returned offsets and scales are not converted back.
    - Define the intended unit contract for `pre_scaler` and adjust returned offset/scale values accordingly.
    - Add a test that changing `pre_scaler` does not silently change physical calibration units.
