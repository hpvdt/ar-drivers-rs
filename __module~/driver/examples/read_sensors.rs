// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::any_glasses_or_dummy;
use ar_drivers::fusion::{rub_to_frd, FusionState, MagCalibrationResult};
use ar_drivers::GlassesEvent;

fn main() {
    let mut glasses = any_glasses_or_dummy().unwrap();
    let serial = glasses.serial().unwrap();
    println!("Got glasses, serial={}", serial);

    let mut fusion = FusionState::new(glasses);

    loop {
        let event = fusion.glasses.read_event().unwrap();

        match event {
            GlassesEvent::AccGyro {
                accelerometer,
                gyroscope,
                timestamp,
            } => {
                let acc_frd = rub_to_frd(&accelerometer);
                let gyr_frd = rub_to_frd(&gyroscope);
                println!(
                    "AccGyro FRD: accelerometer=[x={:+10.4}, y={:+10.4}, z={:+10.4}] gyroscope=[x={:+10.4}, y={:+10.4}, z={:+10.4}] timestamp={:>12}",
                    acc_frd.x,
                    acc_frd.y,
                    acc_frd.z,
                    gyr_frd.x,
                    gyr_frd.y,
                    gyr_frd.z,
                    timestamp
                );
                println!("  - converted from raw {:?}", event);
            }
            GlassesEvent::Magnetometer {
                magnetometer,
                timestamp,
            } => {
                let mag_frd = rub_to_frd(&magnetometer);
                println!(
                    "Magnetometer FRD: mag=[x={:+10.4}, y={:+10.4}, z={:+10.4}] timestamp={:>12}",
                    mag_frd.x, mag_frd.y, mag_frd.z, timestamp
                );
                println!("  - converted from raw {:?}", event);
                match fusion.magCalibrator.evaluate_correct(mag_frd, None, timestamp) {
                    Ok(MagCalibrationResult {
                        quality,
                        direction: Some(direction),
                    }) => println!(
                        "Magnetometer FRD (Calibrated, quality={:.3}): [x={:+10.4}, y={:+10.4}, z={:+10.4}]",
                        quality.confidence, direction.x, direction.y, direction.z
                    ),
                    Ok(MagCalibrationResult { quality, .. }) => println!(
                        "Magnetometer calibration pending: quality={:.3}",
                        quality.confidence
                    ),
                    Err(cause) => println!("Magnetometer calibration unavailable: {:?}", cause),
                }
            }
            _ => {
                println!("Raw event: {:?}", event);
            } // do nothing
        }
    }
}
