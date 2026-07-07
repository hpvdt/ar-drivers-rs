// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::any_glasses_or_dummy;
use ar_drivers::fusion::{rub_to_frd, BadMagDataCause, FusionState};
use ar_drivers::GlassesEvent;
use nalgebra::Vector3;

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
                    "AccGyro FRD: accelerometer={} gyroscope={} timestamp={:>12}",
                    format_vec3(&acc_frd),
                    format_vec3(&gyr_frd),
                    timestamp
                );
                println!("  - converted from raw {}", format_event(&event));
            }
            GlassesEvent::Magnetometer {
                magnetometer,
                timestamp,
            } => {
                let mag_frd = rub_to_frd(&magnetometer);
                println!(
                    "Magnetometer FRD: mag={} timestamp={:>12}",
                    format_vec3(&mag_frd),
                    timestamp
                );
                match fusion.getCalibratedMag(mag_frd) {
                    Ok(calibrated) => {
                        println!("  calibrated (normalized): {}", format_vec3(&calibrated))
                    }
                    Err(cause) => {
                        println!(
                            "  calibration unavailable: {}",
                            format_bad_mag_data_cause(&cause)
                        )
                    }
                }
                println!("  - converted from raw {}", format_event(&event));
            }
            _ => {
                println!("Raw event: {}", format_event(&event));
            } // do nothing
        }
    }
}

fn format_float(value: f32) -> String {
    format!("{:+10.4}", value)
}

fn format_vec3(vector: &Vector3<f32>) -> String {
    format!(
        "[x={}, y={}, z={}]",
        format_float(vector.x),
        format_float(vector.y),
        format_float(vector.z)
    )
}

fn format_event(event: &GlassesEvent) -> String {
    match event {
        GlassesEvent::AccGyro {
            accelerometer,
            gyroscope,
            timestamp,
        } => format!(
            "AccGyro {{ accelerometer: {}, gyroscope: {}, timestamp: {:>12} }}",
            format_vec3(accelerometer),
            format_vec3(gyroscope),
            timestamp
        ),
        GlassesEvent::Magnetometer {
            magnetometer,
            timestamp,
        } => format!(
            "Magnetometer {{ magnetometer: {}, timestamp: {:>12} }}",
            format_vec3(magnetometer),
            timestamp
        ),
        _ => format!("{:?}", event),
    }
}

fn format_bad_mag_data_cause(cause: &BadMagDataCause) -> String {
    match cause {
        BadMagDataCause::InsufficientSamples => "InsufficientSamples".to_string(),
        BadMagDataCause::NumericallyUnstableCalibration { offset, scale } => format!(
            "NumericallyUnstableCalibration {{ offset: {}, scale: {} }}",
            format_vec3(offset),
            format_vec3(scale)
        ),
        BadMagDataCause::WeakCalibratedReading { norm, min_norm } => format!(
            "WeakCalibratedReading {{ norm: {}, min_norm: {} }}",
            format_float(*norm),
            format_float(*min_norm)
        ),
    }
}
