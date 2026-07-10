# AGENTS.md - Fusion Module Guide

## Reference Frames

The library uses multiple coordinate reference frames:

- **RUB (Right-Up-Back)**: Android sensor coordinate system (used in raw sensor data)
- **FRD (Forward-Right-Down)**: Aerospace standard frame (used in fusion outputs)
- **Custom frames**: Configurable via AHRS for different applications

## Coordinate Transformations

- Treat the shared sensor-event documentation and the fusion module as the source
  of truth for reference frames and units. Device events currently use RUB, while
  fusion state and outputs use FRD.
- Keep frame transformations explicit and centralized, document device-specific
  deviations, and use the repository's existing linear-algebra types.
