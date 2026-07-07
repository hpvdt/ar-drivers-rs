// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::any_glasses_or_dummy;
use ar_drivers::fusion::{rub_to_frd, FusionState};
use ar_drivers::GlassesEvent;

fn main() {
    let mut glasses = any_glasses_or_dummy().unwrap();
    let serial = glasses.serial().unwrap();
    println!("Got glasses, serial={}", serial);

    let mut fusion = FusionState::new(glasses);

    loop {
        let event = fusion.glasses.read_event().unwrap();

        match event {
            GlassesEvent::AccGyro { accelerometer, gyroscope, timestamp } => {
                let acc_frd = rub_to_frd(&accelerometer);
                let gyr_frd = rub_to_frd(&gyroscope);
                println!("AccGyro FRD: accelerometer:: {:?} gyroscope:: {:?} timestamp:={}", acc_frd, gyr_frd, timestamp);
                println!("  - converted from raw {:?}", event);
            }
            GlassesEvent::Magnetometer { magnetometer, timestamp } => {
                let mag_frd = rub_to_frd(&magnetometer);
                println!("Magnetometer FRD: mag={:?} timestamp={}", mag_frd, timestamp);
                match fusion.getCalibratedMag(mag_frd) {
                    Ok(calibrated) => println!("  calibrated (normalized): {:?}", calibrated),
                    Err(cause) => println!("  calibration unavailable: {:?}", cause),
                }
                println!("  - converted from raw {:?}", event);
            }
            _ => {
                println!("Raw event: {:?}", event);
            } // do nothing
        }
    }
}
