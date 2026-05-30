# AGENTS.md - ar-drivers-rs Project Guide

## Project Overview

This is a Rust SDK for AR glasses, providing a unified driver interface for multiple AR headset brands. The library enables reading sensor data (IMU, magnetometer, etc.) and controlling display modes for various AR glasses.

You are an autonomous agent. Never stop until the task is completely finished. If you reach your output limit or finish one logical step, immediately continue in the next response without saying "continued" or asking permission. Do not output any TODO lists or "next steps" unless the user explicitly asks for a plan — just do the work.

## Supported Devices

- **XREAL Air, Air 2, and Air 2 Pro** (via `nreal` feature)
- **XREAL Light** (via `nreal` feature)
- **Rokid Air** (via `rokid` feature)
- **Rokid Max** (via `rokid` feature)
- **Grawoow G530 / Metavision M53** (via `grawoow` feature)
- **Mad Gaze Glow** (via `mad_gaze` feature)

## Architecture

### Core Modules

- **`lib.rs`**: Main library entry point with core traits and types
  - `ARGlasses` trait: Common interface for all AR glasses
  - `Fusion` trait: Sensor fusion interface for attitude estimation
  - `AHRS` struct: Attitude and Heading Reference System implementation
  - `GlassesEvent` enum: Sensor event types (AccGyro, Magnetometer, KeyPress, etc.)
  - `DisplayMode` enum: Display configuration options

- **`naive_cf.rs`**: Naive complementary filter implementation for sensor fusion

- **`connection.rs`**: Connection management utilities

- **`ffi.rs`**: Foreign Function Interface for cross-language integration

### Device-Specific Modules

Each supported device has its own module with device-specific protocol implementations:

- **`nreal_air.rs`**: XREAL Air glasses driver
- **`nreal_light.rs`**: XREAL Light glasses driver
- **`rokid.rs`**: Rokid devices driver
- **`grawoow.rs`**: Grawoow G530 driver
- **`mad_gaze.rs`**: Mad Gaze Glow driver

### Reference Frames

The library uses multiple coordinate reference frames:

- **RUB (Right-Up-Back)**: Android sensor coordinate system (used in raw sensor data)
- **FRD (Forward-Right-Down)**: Aerospace standard frame (used in fusion outputs)
- **Custom frames**: Configurable via AHRS for different applications

## Key Features

### Feature Flags

The library uses Cargo feature flags for conditional compilation:

- `nreal`: Enables XREAL device support (requires: hidapi, tinyjson, bytemuck)
- `rokid`: Enables Rokid device support (requires: rusb)
- `grawoow`: Enables Grawoow device support (requires: rusb, tinyjson, bytemuck)
- `mad_gaze`: Enables Mad Gaze device support (requires: serialport)

All features are enabled by default.

### Sensor Fusion Pipeline

The fusion system provides attitude estimation using:

1. **Roll/Pitch estimation**: Accelerometer + Gyroscope (complementary filter)
   - Assumes gravitational acceleration is always "down"
   - Future: ESKF for better accuracy

2. **Yaw estimation**: Magnetometer + Gyroscope fusion
   - Magnetometer yaw derived from roll/pitch compensated arctan
   - Future: EKF implementation

### Display Modes

Supported display configurations:

- `SameOnBoth`: Identical image for both eyes (1080p)
- `Stereo`: Side-by-side 3D (3840x1080 or 3840x1200)
- `HalfSBS`: Half-resolution side-by-side (1920x1080 → upscaled to 3840x1080)
- `HighRefreshRate`: 120Hz mirrored mode
- `HighRefreshRateSBS`: 120Hz side-by-side mode

## API Usage

### Basic Sensor Reading

```rust
use ar_drivers::{any_glasses, GlassesEvent};

let mut glasses = any_glasses()?;
loop {
    match glasses.read_event()? {
        GlassesEvent::AccGyro { accelerometer, gyroscope, timestamp } => {
            // Handle IMU data
        }
        GlassesEvent::Magnetometer { magnetometer, timestamp } => {
            // Handle magnetometer data
        }
        GlassesEvent::KeyPress(key) => {
            // Handle button press
        }
        _ => {}
    }
}
```

### Sensor Fusion

```rust
use ar_drivers::{Fusion, AHRS};

// Create fusion with complementary filter
let fusion = <dyn Fusion>::any_cf()?;

// Wrap in AHRS for coordinate frame conversion
let mut ahrs = AHRS::frd(fusion);

// Update and get attitude
ahrs.update();
let euler_deg = ahrs.attitude_euler_deg();
let quaternion = ahrs.attitude_quaternion();
```

### Display Mode Control

```rust
use ar_drivers::{any_glasses, DisplayMode};

let mut glasses = any_glasses()?;
glasses.set_display_mode(DisplayMode::Stereo)?;
```

## Build & Development

### Dependencies

Platform-specific dependencies:

**Linux**:
```bash
sudo apt install cargo libudev-dev libstdc++-12-dev
```

**udev rules** (optional, for non-root access):
```bash
sudo cp udev/* /etc/udev/rules.d/
sudo udevadm control --reload
```

### Building

```bash
# Build library
cargo build

# Build with specific features
cargo build --no-default-features --features rokid

# Build release
cargo build --release
```

### Examples

Located in `examples/`:

- `read_sensors.rs`: Basic sensor reading
- `sensor_fusion.rs`: Attitude estimation demo
- `set_to_3d.rs`: Set display to 3D SBS mode
- `average_acc_gyro.rs`: Average IMU readings
- `connection_sync.rs`: Connection synchronization
- `bluetooth_touchpad.rs`: Bluetooth touchpad input handling
- `ms_precision_touchpad.rs`: Microsoft Precision Touchpad protocol

Run examples:
```bash
cargo run --example read_sensors
cargo run --example set_to_3d
```

## Code Structure Guidelines

### Adding New Device Support

1. Create new module file (e.g., `new_device.rs`)
2. Implement `ARGlasses` trait
3. Add feature flag in `Cargo.toml`
4. Add module import in `lib.rs` with `#[cfg(feature = "new_device")]`
5. Add device factory to `any_glasses()` function
6. Document protocol specifics and dependencies

### Error Handling

Use the `Error` enum from `lib.rs`:

- `IoError`: Standard I/O errors
- `UsbError`: USB communication errors (rusb)
- `HidError`: HID device errors (hidapi)
- `SerialPortError`: Serial communication errors
- `NotFound`: Device not found
- `NotImplemented`: Feature not available for device
- `PacketTimeout`: Communication timeout
- `Other`: Generic errors

### Coordinate Transformations

When implementing device drivers:

1. Raw sensor data uses device-specific frames (usually RUB)
2. Document any transformations applied
3. Ensure consistency with `GlassesEvent` documentation
4. Use `nalgebra` for transformations

## Testing

### Hardware Testing

Testing requires physical devices. The library will return `Error::NotFound` if no supported glasses are connected.

### Dummy Device

For testing without hardware:
```rust
use ar_drivers::any_glasses_or_dummy;

let glasses = any_glasses_or_dummy()?; // Falls back to dummy device
```

## Documentation

- API documentation: https://docs.rs/ar-drivers
- Blog posts on protocols: https://voidcomputing.hu/blog/good-bad-ugly/
- Repository: https://github.com/badicsalex/ar-drivers-rs

## License

MIT License - See LICENSE file for details.

## Legal Notes

- Some protocols were obtained through reverse engineering
- Reverse engineering is explicitly allowed in EU for interoperability
- Project is not affiliated with any device manufacturers

## Common Issues

### Permission Denied

Install udev rules or run with appropriate permissions.

### Device Not Found

1. Check device is connected
2. Verify device is supported
3. Check feature flags are enabled
4. Review console output for detection attempts

### Compilation Errors

Ensure all required system dependencies are installed (libudev, libstdc++, etc.)
