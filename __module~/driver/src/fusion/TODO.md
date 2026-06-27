# Fusion TODO

- Add a `MagCalibrator` correction wrapper so fusion code does not duplicate offset/scale application.
- Add readiness or introspection for filled calibration samples before solving.
- Cache the last good magnetometer calibration and continue using it when a later solve fails.
- Validate scale and finite values before applying a calibration result, with explicit fallback behavior.
