// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use std::path::Path;

use ar_drivers::fusion::{rub_to_frd, CalibrationQuality, MagCalibrationResult, MagCalibrator};
use ar_drivers::nreal_air::NrealAirReplay;
use ar_drivers::{ARGlasses, GlassesEvent};
use nalgebra::Vector3;

fn format_quality(quality: CalibrationQuality) -> String {
    format!(
        "confidence={:.3}, coverage={:.3}, fitness={:.3}, radial_fitness={:.3}, gravity_fitness={:.3}",
        quality.confidence,
        quality.coverage,
        quality.fitness,
        quality.radial_fitness,
        quality.gravity_fitness
    )
}

fn component_min(left: CalibrationQuality, right: CalibrationQuality) -> CalibrationQuality {
    CalibrationQuality {
        confidence: left.confidence.min(right.confidence),
        coverage: left.coverage.min(right.coverage),
        fitness: left.fitness.min(right.fitness),
        radial_fitness: left.radial_fitness.min(right.radial_fitness),
        gravity_fitness: left.gravity_fitness.min(right.gravity_fitness),
    }
}

fn component_max(left: CalibrationQuality, right: CalibrationQuality) -> CalibrationQuality {
    CalibrationQuality {
        confidence: left.confidence.max(right.confidence),
        coverage: left.coverage.max(right.coverage),
        fitness: left.fitness.max(right.fitness),
        radial_fitness: left.radial_fitness.max(right.radial_fitness),
        gravity_fitness: left.gravity_fitness.max(right.gravity_fitness),
    }
}

#[test]
fn air1_trace_reports_calibration_quality() {
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
    let mut accgyro_samples = 0usize;
    let mut magnetic_samples = 0usize;
    let mut published_samples = 0usize;
    let mut unavailable_samples = 0usize;
    let mut final_quality = None;
    let mut minimum_quality = None;
    let mut maximum_quality = None;

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
                match calibrator.evaluate_correct(rub_to_frd(&magnetometer), gravity, timestamp) {
                    Ok(MagCalibrationResult { quality, direction }) => {
                        published_samples += usize::from(direction.is_some());
                        final_quality = Some(quality);
                        minimum_quality = Some(
                            minimum_quality
                                .map_or(quality, |current| component_min(current, quality)),
                        );
                        maximum_quality = Some(
                            maximum_quality
                                .map_or(quality, |current| component_max(current, quality)),
                        );
                    }
                    Err(_) => unavailable_samples += 1,
                }
            }
            _ => {}
        }
    }

    let final_quality = final_quality.expect("trace produced no calibration quality result");
    let minimum_quality = minimum_quality.unwrap();
    let maximum_quality = maximum_quality.unwrap();
    let duration_us = last_timestamp.unwrap() - first_timestamp.unwrap();
    eprintln!(
        "Air 1 replay: duration_us={duration_us}, accgyro_samples={accgyro_samples}, magnetic_samples={magnetic_samples}, published_samples={published_samples}, unavailable_samples={unavailable_samples}"
    );
    eprintln!("Air 1 final quality: {}", format_quality(final_quality));
    eprintln!(
        "Air 1 component minima: {}",
        format_quality(minimum_quality)
    );
    eprintln!(
        "Air 1 component maxima: {}",
        format_quality(maximum_quality)
    );

    assert!(accgyro_samples > 0);
    assert!(magnetic_samples > 0);
}
