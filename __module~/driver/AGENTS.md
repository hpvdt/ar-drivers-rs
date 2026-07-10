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

### Device-Specific Modules

Each supported device has its own module with device-specific protocol implementations:

- **`nreal_air.rs`**: XREAL Air glasses driver
- **`nreal_light.rs`**: XREAL Light glasses driver
- **`rokid.rs`**: Rokid devices driver
- **`grawoow.rs`**: Grawoow G530 driver
- **`mad_gaze.rs`**: Mad Gaze Glow driver

### Feature Flags

The library uses Cargo feature flags for conditional compilation:

- `nreal`: Enables XREAL device support (requires: hidapi, tinyjson, bytemuck)
- `rokid`: Enables Rokid device support (requires: rusb)
- `grawoow`: Enables Grawoow device support (requires: rusb, tinyjson, bytemuck)
- `mad_gaze`: Enables Mad Gaze device support (requires: serialport)

All features are enabled by default.

### Reference Frames

The library uses multiple coordinate reference frames:

- **RUB (Right-Up-Back)**: Android sensor coordinate system (used in raw sensor data)
- **FRD (Forward-Right-Down)**: Aerospace standard frame (used in fusion outputs)
- **Custom frames**: Configurable via AHRS for different applications

### Display Modes

Supported display configurations:

- `SameOnBoth`: Identical image for both eyes (1080p)
- `Stereo`: Side-by-side 3D (3840x1080 or 3840x1200)
- `HalfSBS`: Half-resolution side-by-side (1920x1080 → upscaled to 3840x1080)
- `HighRefreshRate`: 120Hz mirrored mode
- `HighRefreshRateSBS`: 120Hz side-by-side mode

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

Run examples:
```bash
cargo run --example file_name
```

All examples are located in `examples/`

## Rust Code Style

### Formatting and Imports

- Let `rustfmt` define layout. Follow the repository's `rustfmt.toml` when one is
  present; do not align fields, arguments, or comments by hand.
- Use the line-ending style configured by the repository.
- Group imports consistently: standard library, external crates, then
  `crate`/`super`/`self`. Let rustfmt sort names within each group.
- Import the concrete types and traits used by the module. Avoid glob imports.
- Prefer one module-level import over repeated fully qualified paths when that
  makes the code easier to read, but retain qualification when it clarifies an
  uncommon error or platform type.

### Naming and API Shape

- Use standard Rust naming: `snake_case` for functions, methods, modules, fields,
  and locals; `UpperCamelCase` for types and traits; `SCREAMING_SNAKE_CASE` for
  constants.
- Preserve established public or ABI names when changing them would break a
  caller, but do not copy legacy naming inconsistencies into new APIs.
- Use `Self` in constructors and inherent implementations. Implement `Default`
  when there is one unsurprising baseline configuration, and make `new()`
  delegate to it where appropriate.
- Builder-style configuration methods take and return `self`; state-changing
  operations take `&mut self`; read-only operations take `&self`.

### Documentation and Comments

- Give public items useful `///` doc comments. Use `//!` for module-level behavior
  or constraints.
- Document observable behavior: units, blocking behavior, feature or platform
  availability, error conditions, panics, and safety requirements when relevant.
- Explain non-obvious constraints, magic values, numerical thresholds, and
  invariants. Do not add comments that merely restate the code.
- Keep existing copyright, attribution, and source notes intact when editing a
  file.

### Errors and Control Flow

- Return `Result` from fallible APIs. Use a shared error type when callers need a
  stable error surface, add `From` conversions for reusable lower-level errors,
  and use `?` to preserve the original cause.
- Use specific typed error variants when callers need to distinguish recovery
  paths. Preserve error meaning and message wording during refactors when callers
  may depend on them.
- Do not use `unwrap()` or `expect()` for ordinary runtime failures in library
  code. They are acceptable in tests and small examples, or for a locally proven
  invariant whose reason is clear.
- Prefer early returns for invalid inputs and guard conditions. Use `match` when
  every enum case matters or when it expresses branching more clearly.
- Do not silently discard errors. If best-effort processing intentionally
  continues, keep enough context to diagnose the failure.

### Modules and Conditional Compilation

- Keep modules cohesive and give each one a clear responsibility. Split a module
  when its responsibilities or private implementation details stop being related.
- Keep helpers private by default. Expose `pub(crate)` for genuine cross-module
  internals and `pub` only for public API needed by downstream users.
- Keep `#[cfg(...)]` gates next to the module, import, implementation, or function
  they control. Optional functionality and its dependencies should be guarded by
  the same feature.
- Keep behavior consistent across platform-specific implementations when the
  public API is shared.

### Types and Data Handling

- Prefer domain types and newtypes when they prevent invalid combinations of
  primitive values. Use exact-width integers for binary formats and external
  interfaces whose widths are fixed.
- Give repeated constants and thresholds descriptive names. Include units in a
  name or doc comment when the type alone cannot express them.
- Make byte order explicit when reading or writing binary data. Validate lengths,
  tags, ranges, and conversions before indexing, slicing, or casting bytes.
- Preserve numerical precision deliberately. Reject non-finite or degenerate data
  before normalization, division, decomposition, or other sensitive operations.
- Prefer iterators when they make the transformation clearer; use loops when
  control flow, mutation, or early exit is easier to understand that way.

### Concurrency, FFI, and Unsafe Code

- Keep lock acquisition and thread lifecycle logic centralized. Propagate poison
  and join failures rather than introducing new panics.
- Use atomics with an explicitly chosen ordering; keep the ordering decision in
  one named constant when multiple operations share it.
- Avoid `unsafe` when a safe abstraction is practical. Keep unavoidable unsafe
  blocks and unsafe impls as small as possible, and add a `SAFETY:` comment that
  states the invariant being upheld.
- Treat exported symbol names, signatures, layouts, ownership, and lifetimes as
  ABI. Do not change them without coordinating and testing all callers.
- Never allow a panic to unwind across an `extern "C"` boundary. Validate raw
  pointers and lengths before dereferencing, and document caller obligations.
- Use an explicit representation such as `#[repr(C)]` when a type's layout is
  shared across an FFI or binary boundary.

### Tests and Examples

- Put focused unit tests near the implementation under `#[cfg(test)]`; use
  integration tests for behavior exercised through the public API.
- Name tests after observable behavior and cover success, malformed input,
  boundary values, and error variants.
- Prefer deterministic tests and local fixtures. Keep tests that require external
  resources, timing, or environment state clearly separate and document their
  prerequisites.
- Compare floating-point results with a tolerance derived from the algorithm;
  use exact equality only for values that are constructed exactly.
- Examples may use `unwrap()` to stay concise, but should demonstrate the public
  API and avoid becoming alternate implementations of library logic.

### Validation

For Rust changes, run the narrowest relevant checks first, then broaden them:

```bash
cargo fmt --all -- --check
cargo check --all-targets --all-features
cargo test --all-targets --all-features
cargo clippy --all-targets --all-features -- -D warnings
```

Adapt feature flags and targets when a project does not support building every
combination together. Run narrower package, module, or test checks first for fast
feedback, but complete the broad checks applicable to the repository before
submitting a change.

## Code Structure Guidelines

### Adding New Device Support

1. Create new module file (e.g., `new_device.rs`)
2. Implement `ARGlasses` trait
3. Add feature flag in `Cargo.toml`
4. Add module import in `lib.rs` with `#[cfg(feature = "new_device")]`
5. Add device factory to `any_glasses()` function
6. Document protocol specifics and dependencies

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

## Protocol Blog Posts

https://voidcomputing.hu/blog/good-bad-ugly/
https://voidcomputing.hu/blog/worse-better-prettier/

## Common Issues

### Permission Denied

Install udev rules or run with appropriate permissions.

### Compilation Errors

Ensure all required system dependencies are installed (libudev, libstdc++, etc.)
