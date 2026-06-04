#!/usr/bin/env bash

cargo clean

echo "[Start Compiling ... Zig compiler and cargo-zigbuild must be installed]"

cargo zigbuild --target x86_64-pc-windows-gnu --release
cargo zigbuild --target aarch64-apple-darwin --release

# TODO: this will fail on OS without libudev (I mean linux)
cargo zigbuild --target x86_64-unknown-linux-gnu --release || \
  cargo zigbuild --target x86_64-unknown-linux-gnu --release --no-default-features --features rokid,nreal,grawoow