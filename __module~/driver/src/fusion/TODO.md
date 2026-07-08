
## Tasks

- [ ] Accept valid samples with zero components
    - `evaluate_sample_vec` currently rejects any component that is not `is_normal()`.
    - Replace this with a check that rejects `NaN`/infinite values and near-zero field vectors, while allowing valid finite zero components.
    - Add coverage for samples like `[45.0, 0.0, -12.0]`.

- [ ] Return calibration values in caller-visible units
    - Samples are divided by `pre_scaler` before storage, but returned offsets and scales are not converted back.
    - Define the intended unit contract for `pre_scaler` and adjust returned offset/scale values accordingly.
    - Add a test that changing `pre_scaler` does not silently change physical calibration units.
