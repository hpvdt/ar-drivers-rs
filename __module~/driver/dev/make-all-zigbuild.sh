#!/usr/bin/env bash

cargo clean

echo "[Start Compiling ... Zig compiler and cargo-zigbuild must be installed]"

cargo zigbuild --target x86_64-pc-windows-gnu --release || echo "WARNING: Failed to build for x86_64-pc-windows-gnu"
cargo zigbuild --target aarch64-apple-darwin --release || echo "WARNING: Failed to build for aarch64-apple-darwin"

# TODO: this will fail on OS without libudev (I mean linux)
cargo zigbuild --target x86_64-unknown-linux-gnu --release || \
  cargo zigbuild --target x86_64-unknown-linux-gnu --release --no-default-features --features rokid,nreal,grawoow

echo "[Copying to Plugins ...]"
[ -f target/x86_64-pc-windows-gnu/release/ar_drivers.dll ] && cp target/x86_64-pc-windows-gnu/release/ar_drivers.dll ../../Plugins/
[ -f target/aarch64-apple-darwin/release/libar_drivers.dylib ] && cp target/aarch64-apple-darwin/release/libar_drivers.dylib ../../Plugins/
cp target/x86_64-unknown-linux-gnu/release/libar_drivers.so ../../Plugins/

