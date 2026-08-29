// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::any_glasses_or_dummy;
use ar_drivers::fusion::{
    rub_to_frd, CalibrationQuality, FusionState, MagCalibrationResult, MagCalibrator,
};
use ar_drivers::GlassesEvent;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

fn format_quality(quality: &CalibrationQuality) -> String {
    format!(
        "confidence={:.3}, coverage={:.3}, fitness={:.3}, radial_fitness={:.3}, gravity_fitness={:.3}",
        quality.confidence,
        quality.coverage,
        quality.fitness,
        quality.radial_fitness,
        quality.gravity_fitness
    )
}

fn main() {
    let dump_config = std::env::args().any(|arg| arg == "--dump-config");
    let probe = std::env::args().any(|arg| arg == "--probe");
    if dump_config {
        let nreal = ar_drivers::nreal_air::NrealAir::new().unwrap();
        println!("cfg: {:?}", nreal.get_config_json()["IMU"]);
        return;
    }
    if probe {
        probe_mappings();
        return;
    }
    let mut glasses = any_glasses_or_dummy().unwrap();
    let serial = glasses.serial().unwrap();
    println!("Got glasses, serial={}", serial);

    let mut fusion = FusionState::new(glasses);
    let mut gravity: Option<Vector3<f32>> = None;

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
                // felt acceleration is the body-frame gravity reference direction
                if let Some(direction) = acc_frd.try_normalize(0.0) {
                    gravity = Some(direction);
                }
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
                match fusion
                    .magCalibrator
                    .evaluate_correct(mag_frd, gravity, timestamp)
                {
                    Ok(MagCalibrationResult {
                        quality,
                        direction: Some(direction),
                    }) => println!(
                        "Magnetometer FRD (Calibrated, {}): [x={:+10.4}, y={:+10.4}, z={:+10.4}]",
                        format_quality(&quality),
                        direction.x,
                        direction.y,
                        direction.z
                    ),
                    Ok(MagCalibrationResult { quality, .. }) => println!(
                        "Magnetometer calibration pending: {}",
                        format_quality(&quality)
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

/// Probe of all 24 signed axis mappings from raw to RUB. For each of the 6
/// permutations, all 8 sign combinations are candidates, tracked by its own
/// calibrator and ranked every 500 mag samples by radial+gravity fitness.
/// The driver's RUB emission provides inverse lookup of raw components.
fn probe_mappings() {
    let combs: Vec<[usize; 3]> = vec![
        [0, 1, 2],
        [1, 2, 0],
        [2, 0, 1],
        [0, 2, 1],
        [1, 0, 2],
        [2, 1, 0],
    ];
    struct Candidate {
        label: String,
        perm: [usize; 3],
        signs: [usize; 3],
    }
    let labels_all: Vec<Candidate> = combs
        .iter()
        .flat_map(|perm| {
            (0..8usize).map(move |mask| Candidate {
                label: format!(
                    "{}{},{}{},{}{}",
                    if mask & 4 == 0 { "+" } else { "-" },
                    "xyz".chars().nth(perm[0]).unwrap(),
                    if mask & 2 == 0 { "+" } else { "-" },
                    "xyz".chars().nth(perm[1]).unwrap(),
                    if mask & 1 == 0 { "+" } else { "-" },
                    "xyz".chars().nth(perm[2]).unwrap(),
                ),
                perm: *perm,
                signs: [mask >> 2 & 1, mask >> 1 & 1, mask & 1],
            })
        })
        .collect();
    let mut labels: Vec<String> = labels_all.iter().map(|c| c.label.clone()).collect();
    let mut candidates: Vec<([usize; 3], [f32; 3])> = labels_all
        .iter()
        .map(|c| {
            (
                c.perm,
                [
                    if c.signs[0] == 0 { 1.0 } else { -1.0 },
                    if c.signs[1] == 0 { 1.0 } else { -1.0 },
                    if c.signs[2] == 0 { 1.0 } else { -1.0 },
                ],
            )
        })
        .collect();
    let mut calibrators: Vec<MagCalibrator<1023>> =
        (0..candidates.len()).map(|_| MagCalibrator::new()).collect();
    // gyro_q_mag dumped from the device (JPL per the device's own comment).
    let q_mag = UnitQuaternion::from_quaternion(Quaternion::new(
        0.612372, 0.353553, 0.612372, 0.353553,
    ));
    let gyro_map = |v: Vector3<f32>| Vector3::new(-v.x, v.z, v.y);
    let quat_candidates: Vec<(String, Box<dyn Fn(Vector3<f32>) -> Vector3<f32>>)> = vec![
        ("Q1: G o R(q)".to_string(), Box::new(move |v| gyro_map(q_mag * v))),
        (
            "Q2: G o R(q)^-1".to_string(),
            Box::new(move |v| gyro_map(q_mag.inverse() * v)),
        ),
    ];
    let mut quat_calibrators: Vec<MagCalibrator<1023>> =
        (0..quat_candidates.len()).map(|_| MagCalibrator::new()).collect();
    let mut glasses = any_glasses_or_dummy().unwrap();
    let mut gravity: Option<Vector3<f32>> = None;
    let mut summary_count: usize = 0;
    let mut best_quality: Vec<CalibrationQuality> = (0..candidates.len())
        .map(|_| CalibrationQuality {
            confidence: 0.0,
            coverage: 0.0,
            fitness: 0.0,
            radial_fitness: 0.0,
            gravity_fitness: 0.0,
        })
        .collect();
    loop {
        let event = glasses.read_event().unwrap();
        match event {
            GlassesEvent::AccGyro { accelerometer, .. } => {
                let acc_frd = rub_to_frd(&accelerometer);
                if let Some(direction) = acc_frd.try_normalize(0.0) {
                    gravity = Some(direction);
                }
            }
            GlassesEvent::Magnetometer {
                magnetometer,
                timestamp,
            } => {
                // Inverse of the driver's current RUB emission (-y, z, x):
                // raw.x = event.z, raw.y = -event.x, raw.z = event.y.
                let raw = Vector3::new(magnetometer.z, -magnetometer.x, magnetometer.y);
                let parts = [raw.x, raw.y, raw.z];
                for (idx, (perm, signs)) in candidates.iter().enumerate() {
                    let candidate = Vector3::new(
                        parts[perm[0]] * signs[0],
                        parts[perm[1]] * signs[1],
                        parts[perm[2]] * signs[2],
                    );
                    let mag_frd = rub_to_frd(&candidate);
                    if let Ok(MagCalibrationResult { quality, .. }) =
                        calibrators[idx].evaluate_correct(mag_frd, gravity, timestamp)
                    {
                        best_quality[idx] = quality;
                    }
                }
                summary_count += 1;
                if summary_count % 500 == 0 {
                    write_summary(&labels, &best_quality, summary_count);
                }
            }
            _ => {}
        }
    }
}

fn write_summary(labels: &[String], qualities: &[CalibrationQuality], count: usize) {
    let mut order: Vec<usize> = (0..labels.len()).collect();
    order.sort_by(|&a, &b| {
        (qualities[b].gravity_fitness + qualities[b].radial_fitness)
            .partial_cmp(&(qualities[a].gravity_fitness + qualities[a].radial_fitness))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    println!("== summary at {} samples (best first) ==", count);
    for idx in order.iter().take(6) {
        let q = qualities[*idx];
        println!(
            "  {} radial={:.3} gravity={:.3} coverage={:.3}",
            labels[*idx], q.radial_fitness, q.gravity_fitness, q.coverage
        );
    }
}
