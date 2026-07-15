use nalgebra::{Matrix3, Vector3};

use super::bad_mag_cause::BadCalibration;
use super::mag_calibration::MagCalibrator;

#[test]
fn mag_calibrator_solves_synthetic_offset_and_full_spd_correction() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<63>(offset, distortion);

    let (actual_offset, actual_correction) = calibrator.perform_calibration().unwrap();

    assert_vec_close(actual_offset, offset, 0.05);
    assert_matrix_close(actual_correction * distortion, Matrix3::identity(), 0.05);
}

#[test]
fn mag_calibrator_degenerate_data_does_not_panic() {
    let mut calibrator = MagCalibrator::<9>::new();
    for timestamp_us in 0..9 {
        calibrator.evaluate_sample_vec(Vector3::new(5.0, 6.0, 7.0), timestamp_us);
    }

    assert!(matches!(
        calibrator.perform_calibration(),
        Err(BadCalibration::DegenerateSoftIronMatrix { .. })
    ));
}

#[test]
fn mag_calibrator_waits_for_the_full_buffer_after_reaching_the_model_minimum() {
    let mut calibrator = MagCalibrator::<12>::new();
    for i in 0..9 {
        calibrator.evaluate_sample_vec(Vector3::new(5.0 + i as f32, 6.0, 7.0), i as u64);
    }

    assert!(matches!(
        calibrator.perform_calibration(),
        Err(BadCalibration::InsufficientSamples {
            samples: 9,
            required: 12
        })
    ));
}

#[test]
fn mag_calibrator_rejects_nearly_collinear_samples() {
    let mut calibrator = MagCalibrator::<12>::new();
    for i in 0..12 {
        let t = i as f32 * 0.0001;
        calibrator.evaluate_sample_vec(
            Vector3::new(10.0 + t, -5.0 + 2.0 * t, 3.0 + 0.5 * t),
            i as u64,
        );
    }

    assert!(matches!(
        calibrator.perform_calibration(),
        Err(BadCalibration::DegenerateSoftIronMatrix { .. })
    ));
}

#[test]
fn mag_calibrator_clamps_k_and_excludes_self_distance() {
    assert!((mean_distance_after_replacement(0) - 27.5).abs() < 0.001);
    assert!((mean_distance_after_replacement(99) - 51.666668).abs() < 0.001);
}

#[test]
fn mag_calibrator_accepts_zero_components_and_rejects_bad_vectors() {
    let mut calibrator = MagCalibrator::<12>::new();
    for (timestamp_us, sample) in [
        Vector3::new(45.0, 0.0, -12.0),
        Vector3::new(f32::NAN, 1.0, 1.0),
        Vector3::new(f32::INFINITY, 1.0, 1.0),
        Vector3::new(1.0e-8, 0.0, 0.0),
    ]
    .into_iter()
    .enumerate()
    {
        calibrator.evaluate_sample_vec(sample, timestamp_us as u64);
    }

    assert!(matches!(
        calibrator.perform_calibration(),
        Err(BadCalibration::InsufficientSamples {
            samples: 1,
            required: 12
        })
    ));
}

#[test]
fn mag_calibrator_defaults_sample_lifespan_to_one_hour() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<12>(offset, distortion);

    calibrator.evaluate_sample_vec(Vector3::new(20.0, 30.0, 40.0), 60 * 60 * 1_000_000 + 1);

    assert!(matches!(
        calibrator.perform_calibration(),
        Err(BadCalibration::InsufficientSamples {
            samples: 1,
            required: 12
        })
    ));
}

#[test]
fn mag_calibrator_uses_configured_sample_lifespan() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<12>(offset, distortion).max_sample_lifespan_us(10);

    calibrator.evaluate_sample_vec(Vector3::new(20.0, 30.0, 40.0), 11);

    assert!(matches!(
        calibrator.perform_calibration(),
        Err(BadCalibration::InsufficientSamples {
            samples: 1,
            required: 12
        })
    ));
}

fn mean_distance_after_replacement(k: usize) -> f32 {
    let mut calibrator = MagCalibrator::<4>::new().num_neighbors(k);
    for (timestamp_us, x) in [1.0, 11.0, 21.0, 101.0].into_iter().enumerate() {
        calibrator.evaluate_sample_vec(Vector3::new(x, 1.0, 1.0), timestamp_us as u64);
    }
    calibrator.evaluate_sample_vec(Vector3::new(201.0, 1.0, 1.0), 4);
    calibrator.get_mean_distance()
}

fn seeded_calibrator<const N: usize>(
    offset: Vector3<f32>,
    distortion: Matrix3<f32>,
) -> MagCalibrator<N> {
    let mut calibrator = MagCalibrator::new();
    for i in 0..N {
        let direction = sample_direction(i, N);
        calibrator.evaluate_sample_vec(offset + distortion * direction, 0);
    }
    calibrator
}

fn sample_direction(i: usize, n: usize) -> Vector3<f32> {
    let theta = 0.37 + i as f32 * 1.21;
    let z = -0.8 + 1.6 * i as f32 / (n - 1) as f32;
    let radius = (1.0 - z * z).sqrt();
    Vector3::new(radius * theta.cos(), radius * theta.sin(), z)
}

fn assert_vec_close(actual: Vector3<f32>, expected: Vector3<f32>, tolerance: f32) {
    let diff = (actual - expected).norm();
    assert!(
        diff < tolerance,
        "actual={} expected={} diff={}",
        actual.transpose(),
        expected.transpose(),
        diff
    );
}

fn assert_matrix_close(actual: Matrix3<f32>, expected: Matrix3<f32>, tolerance: f32) {
    let diff = (actual - expected).norm();
    assert!(
        diff < tolerance,
        "actual={} expected={} diff={}",
        actual,
        expected,
        diff
    );
}
