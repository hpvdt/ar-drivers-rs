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

        println!("Raw Event: {:?}", event);

        match event {
            GlassesEvent::AccGyro { accelerometer, gyroscope, timestamp } => {
                let acc_frd = rub_to_frd(&accelerometer);
                let gyr_frd = rub_to_frd(&gyroscope);
                println!("AccGyro FRD: acc={:10.7} gyr={:10.7} ts={}", acc_frd.transpose(), gyr_frd.transpose(), timestamp);
            }
            GlassesEvent::Magnetometer { magnetometer, timestamp } => {
                let mag_frd = rub_to_frd(&magnetometer);
                println!("Magnetometer FRD: mag={:10.7} ts={}", mag_frd.transpose(), timestamp);
                match fusion.getCalibratedMag(mag_frd) {
                    Ok(calibrated) => println!("  calibrated (normalized): {:10.7}", calibrated.transpose()),
                    Err(cause) => println!("  calibration unavailable: {:?}", cause),
                }
            }
            _ => {} // do nothing
        }
    }
}
