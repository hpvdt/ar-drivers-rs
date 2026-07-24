use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use super::bad_mag_cause::{BadCalibration, BadMagCause};
use super::mag_calibration::MagCalibrator;

#[test]
fn mag_calibrator_corrects_synthetic_full_spd_distortion() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<63>(offset, distortion);

    for (timestamp_us, expected) in [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, -2.0, 3.0).normalize(),
    ]
    .into_iter()
    .enumerate()
    {
        let corrected = calibrator
            .evaluate_correct(offset + distortion * expected, None, timestamp_us as u64)
            .unwrap();
        assert_vec_close(corrected, expected, 0.05);
    }
}

#[test]
fn mag_calibrator_corrects_asymmetrically_sampled_distortion() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = MagCalibrator::<63>::new();
    for i in 0..63 {
        let theta = 0.37 + i as f32 * 1.21;
        let z = -0.1 + i as f32 / 62.0;
        let radius = (1.0 - z * z).sqrt();
        let direction = Vector3::new(radius * theta.cos(), radius * theta.sin(), z);
        let _ = calibrator.evaluate_correct(offset + distortion * direction, None, i as u64);
    }

    let expected = Vector3::new(1.0, -2.0, -1.0).normalize();
    let corrected = calibrator
        .evaluate_correct(offset + distortion * expected, None, 64)
        .unwrap();

    assert_vec_close(corrected, expected, 0.05);
}

#[test]
fn mag_calibrator_returns_stable_direct_corrections() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<63>(offset, distortion);
    let expected = Vector3::new(1.0, -2.0, 3.0).normalize();
    let raw = offset + distortion * expected;

    let first = calibrator.evaluate_correct(raw, None, 1).unwrap();
    let second = calibrator.evaluate_correct(raw, None, 2).unwrap();

    assert_vec_close(second, first, 0.01);
}

#[test]
fn mag_calibrator_rejects_degenerate_data() {
    let mut calibrator = MagCalibrator::<9>::new();
    let result = (0..9)
        .map(|timestamp_us| {
            calibrator.evaluate_correct(Vector3::new(5.0, 6.0, 7.0), None, timestamp_us)
        })
        .last()
        .unwrap();

    assert!(matches!(
        result,
        Err(BadMagCause::BadCalibration(
            BadCalibration::Unsolveable { .. }
        ))
    ));
}

#[test]
fn mag_calibrator_waits_for_the_full_buffer_after_reaching_the_model_minimum() {
    let mut calibrator = MagCalibrator::<12>::new();
    let mut result = None;
    for i in 0..9 {
        result = Some(calibrator.evaluate_correct(
            Vector3::new(5.0 + i as f32, 6.0, 7.0),
            None,
            i as u64,
        ));
    }

    assert!(matches!(
        result.unwrap(),
        Err(BadMagCause::BadCalibration(
            BadCalibration::InsufficientSamples {
                samples: 9,
                required: 12
            }
        ))
    ));
}

#[test]
fn mag_calibrator_rejects_nearly_collinear_samples() {
    let mut calibrator = MagCalibrator::<12>::new();
    let mut result = None;
    for i in 0..12 {
        let t = i as f32 * 0.0001;
        result = Some(calibrator.evaluate_correct(
            Vector3::new(10.0 + t, -5.0 + 2.0 * t, 3.0 + 0.5 * t),
            None,
            i as u64,
        ));
    }

    assert!(matches!(
        result.unwrap(),
        Err(BadMagCause::BadCalibration(
            BadCalibration::DegenerateSoftIronMatrix { .. }
        ))
    ));
}

#[test]
fn mag_calibrator_keeps_last_correction_after_rejected_refit() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<12>(offset, distortion).max_sample_lifespan_us(0);
    let mut result = None;
    let expected = Vector3::x();
    let raw = offset + distortion * expected;

    for _ in 0..12 {
        result = Some(calibrator.evaluate_correct(raw, None, 1));
    }

    assert_vec_close(result.unwrap().unwrap(), expected, 0.05);
}

#[test]
fn mag_calibrator_clamps_neighbor_count_through_public_result() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let expected = Vector3::new(1.0, -2.0, 3.0).normalize();

    for k in [0, 99] {
        let mut calibrator = seeded_calibrator::<63>(offset, distortion).num_neighbors(k);
        let corrected = calibrator
            .evaluate_correct(offset + distortion * expected, None, 1)
            .unwrap();
        assert_vec_close(corrected, expected, 0.05);
    }
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
        let result = calibrator.evaluate_correct(sample, None, timestamp_us as u64);
        assert!(matches!(
            result,
            Err(BadMagCause::BadCalibration(
                BadCalibration::InsufficientSamples {
                    samples: 1,
                    required: 12
                }
            ))
        ));
    }
}

#[test]
fn mag_calibrator_defaults_sample_lifespan_to_one_hour() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<12>(offset, distortion);

    let result = calibrator.evaluate_correct(
        Vector3::new(20.0, 30.0, 40.0),
        None,
        60 * 60 * 1_000_000 + 1,
    );

    assert!(matches!(
        result,
        Err(BadMagCause::BadCalibration(
            BadCalibration::InsufficientSamples {
                samples: 1,
                required: 12
            }
        ))
    ));
}

#[test]
fn mag_calibrator_uses_configured_sample_lifespan() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<12>(offset, distortion).max_sample_lifespan_us(10);

    let result = calibrator.evaluate_correct(Vector3::new(20.0, 30.0, 40.0), None, 11);

    assert!(matches!(
        result,
        Err(BadMagCause::BadCalibration(
            BadCalibration::InsufficientSamples {
                samples: 1,
                required: 12
            }
        ))
    ));
}

#[test]
fn mag_calibrator_improves_with_consistent_attitudes() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let world = Vector3::new(0.5, 0.3, -0.8).normalize();

    let mut plain = MagCalibrator::<63>::new();
    let mut tagged = MagCalibrator::<63>::new();
    for i in 0..63 {
        let yaw = i as f32 * 2.4;
        let pitch = 0.35 * (i as f32 * 1.7).sin();
        let attitude = UnitQuaternion::from_euler_angles(0.0, pitch, yaw);
        let body = attitude.inverse() * world;
        let raw = offset + distortion * body;
        let _ = plain.evaluate_correct(raw, None, i as u64);
        let _ = tagged.evaluate_correct(raw, Some(attitude), i as u64);
    }

    for probe in [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, -2.0, 3.0).normalize(),
    ] {
        let raw = offset + distortion * probe;
        let err_plain = (plain.evaluate_correct(raw, None, 100).unwrap() - probe).norm();
        let err_tagged = (tagged.evaluate_correct(raw, None, 100).unwrap() - probe).norm();
        assert!(
            err_tagged < err_plain,
            "probe={} err_plain={} err_tagged={}",
            probe.transpose(),
            err_plain,
            err_tagged
        );
        assert!(
            err_tagged < 0.8 * err_plain,
            "probe={} err_plain={} err_tagged={}",
            probe.transpose(),
            err_plain,
            err_tagged
        );
    }
}

#[test]
fn mag_calibrator_tolerates_inconsistent_attitudes() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let world = Vector3::new(0.5, 0.3, -0.8).normalize();

    let mut plain = MagCalibrator::<63>::new();
    let mut misled = MagCalibrator::<63>::new();
    for i in 0..63 {
        let yaw = i as f32 * 2.4;
        let pitch = 0.35 * (i as f32 * 1.7).sin();
        let attitude = UnitQuaternion::from_euler_angles(0.0, pitch, yaw);
        let body = attitude.inverse() * world;
        let raw = offset + distortion * body;
        let _ = plain.evaluate_correct(raw, None, i as u64);
        let wrong = UnitQuaternion::from_euler_angles(i as f32 * 0.7, i as f32 * 1.1, i as f32);
        let _ = misled.evaluate_correct(raw, Some(wrong), i as u64);
    }

    for probe in [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, -2.0, 3.0).normalize(),
    ] {
        let raw = offset + distortion * probe;
        let err_plain = (plain.evaluate_correct(raw, None, 100).unwrap() - probe).norm();
        let err = (misled.evaluate_correct(raw, None, 100).unwrap() - probe).norm();
        assert!(
            err < err_plain + 0.05,
            "probe={} err_plain={} err={} exceeds the bounded-degradation limit",
            probe.transpose(),
            err_plain,
            err
        );
    }
}

#[test]
fn mag_calibrator_zero_attitude_weight_matches_plain() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);

    let mut plain = seeded_calibrator::<63>(offset, distortion);
    let mut weighted = seeded_calibrator::<63>(offset, distortion).attitude_weight(0.0);
    let expected = Vector3::new(1.0, -2.0, 3.0).normalize();
    let raw = offset + distortion * expected;

    let plain_corrected = plain.evaluate_correct(raw, None, 1).unwrap();
    let weighted_corrected = weighted
        .evaluate_correct(raw, Some(UnitQuaternion::identity()), 1)
        .unwrap();

    assert_vec_close(weighted_corrected, plain_corrected, 1.0e-6);
}

fn seeded_calibrator<const N: usize>(
    offset: Vector3<f32>,
    distortion: Matrix3<f32>,
) -> MagCalibrator<N> {
    let mut calibrator = MagCalibrator::new();
    for i in 0..N {
        let direction = sample_direction(i, N);
        let _ = calibrator.evaluate_correct(offset + distortion * direction, None, 0);
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
