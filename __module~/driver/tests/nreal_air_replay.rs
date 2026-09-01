// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use std::path::Path;

use ar_drivers::fusion::{rub_to_frd, MagCalibrationResult, MagCalibrator};
use ar_drivers::nreal_air::NrealAirReplay;
use ar_drivers::{ARGlasses, GlassesEvent};
use nalgebra::Vector3;

const MAX_CALIBRATION_TIME_US: u64 = 60_000_000;
const MIN_AVERAGE_FITNESS: f64 = 0.5;

fn assert_air1_trace_calibrates(use_gravity: bool) {
    let mode = if use_gravity {
        "with_gravity"
    } else {
        "without_gravity"
    };
    let trace = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("nreal_air_air1_60s.log");
    let mut replay = NrealAirReplay::open(&trace).unwrap();
    let mut calibrator = MagCalibrator::<1023>::new();
    let mut gravity: Option<Vector3<f32>> = None;
    let mut previous_timestamp = None;
    let mut first_timestamp = None;
    let mut last_timestamp = None;
    let mut first_calibrated_timestamp = None;
    let mut accgyro_samples = 0usize;
    let mut magnetic_samples = 0usize;
    let mut calibrated_samples = 0usize;
    let mut quality_samples = 0usize;
    let mut radial_fitness_sum = 0.0f64;
    let mut gravity_fitness_sum = 0.0f64;

    loop {
        let event = replay.read_event().unwrap();
        let timestamp = match event {
            GlassesEvent::AccGyro { timestamp, .. }
            | GlassesEvent::Magnetometer { timestamp, .. } => timestamp,
            _ => continue,
        };
        if previous_timestamp.is_some_and(|previous| timestamp < previous) {
            break;
        }
        previous_timestamp = Some(timestamp);
        first_timestamp.get_or_insert(timestamp);
        last_timestamp = Some(timestamp);

        match event {
            GlassesEvent::AccGyro { accelerometer, .. } => {
                gravity = rub_to_frd(&accelerometer).try_normalize(0.0);
                accgyro_samples += 1;
            }
            GlassesEvent::Magnetometer {
                magnetometer,
                timestamp,
            } => {
                magnetic_samples += 1;
                let gravity_hint = if use_gravity {
                    let Some(gravity) = gravity else {
                        continue;
                    };
                    Some(gravity)
                } else {
                    None
                };
                let MagCalibrationResult { quality, direction } = calibrator
                    .evaluate_correct(rub_to_frd(&magnetometer), gravity_hint, timestamp)
                    .unwrap_or_else(|error| {
                        panic!("Air 1 replay {mode} calibration failed at {timestamp}: {error:?}")
                    });

                if direction.is_some() {
                    calibrated_samples += 1;
                    first_calibrated_timestamp.get_or_insert(timestamp);
                }
                if first_calibrated_timestamp.is_some() {
                    quality_samples += 1;
                    radial_fitness_sum += f64::from(quality.radial_fitness);
                    gravity_fitness_sum += f64::from(quality.gravity_fitness);
                }
            }
            _ => {}
        }
    }

    let first_timestamp = first_timestamp.expect("Air 1 trace contained no sensor timestamp");
    let last_timestamp = last_timestamp.unwrap();
    let duration_us = last_timestamp - first_timestamp;
    let first_calibrated_timestamp = first_calibrated_timestamp
        .unwrap_or_else(|| panic!("Air 1 replay {mode} produced no calibrated reading"));
    let first_calibrated_after_us = first_calibrated_timestamp - first_timestamp;
    assert!(
        accgyro_samples > 0,
        "Air 1 replay {mode} had no AccGyro samples"
    );
    assert!(
        magnetic_samples > 0,
        "Air 1 replay {mode} had no magnetic samples"
    );
    assert!(
        calibrated_samples > 0,
        "Air 1 replay {mode} had no calibrated samples"
    );
    assert!(
        quality_samples > 0,
        "Air 1 replay {mode} had no post-publication quality samples"
    );
    assert!(
        first_calibrated_after_us <= MAX_CALIBRATION_TIME_US,
        "Air 1 replay {mode} first calibrated after {first_calibrated_after_us} us"
    );

    let radial_fitness_average = radial_fitness_sum / quality_samples as f64;
    let gravity_fitness_average = gravity_fitness_sum / quality_samples as f64;
    eprintln!(
        "Air 1 replay {mode}: duration_us={duration_us}, accgyro_samples={accgyro_samples}, magnetic_samples={magnetic_samples}, calibrated_samples={calibrated_samples}, first_calibrated_after_us={first_calibrated_after_us}, quality_samples={quality_samples}, radial_fitness_average={radial_fitness_average:.3}, gravity_fitness_average={gravity_fitness_average:.3}"
    );
    assert!(
        radial_fitness_average > MIN_AVERAGE_FITNESS,
        "Air 1 replay {mode} average radial_fitness {radial_fitness_average:.6} must be greater than {MIN_AVERAGE_FITNESS}"
    );
    assert!(
        gravity_fitness_average > MIN_AVERAGE_FITNESS,
        "Air 1 replay {mode} average gravity_fitness {gravity_fitness_average:.6} must be greater than {MIN_AVERAGE_FITNESS}"
    );
}

#[test]
fn air1_trace_calibrates_with_gravity() {
    assert_air1_trace_calibrates(true);
}

#[test]
fn air1_trace_calibrates_without_gravity() {
    assert_air1_trace_calibrates(false);
}
