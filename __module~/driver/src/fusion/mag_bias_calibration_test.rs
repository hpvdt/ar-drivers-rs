use nalgebra::Vector3;

use super::mag_bias_calibration::MagBiasCalibration;

fn assert_vec_close(actual: Vector3<f32>, expected: Vector3<f32>) {
    let diff = (actual - expected).norm();
    assert!(
        diff < 0.0001,
        "actual={} expected={} diff={}",
        actual.transpose(),
        expected.transpose(),
        diff
    );
}

#[test]
fn mag_bias_remains_disabled_before_duration_and_fill() {
    let mut calibration = MagBiasCalibration::<4>::new();

    calibration.record(Vector3::new(10.0, 0.0, 0.0), 0);
    calibration.record(Vector3::new(12.0, 0.0, 0.0), 1_000_000);
    calibration.record(Vector3::new(14.0, 0.0, 0.0), 2_000_000);

    assert!(!calibration.is_enabled());
    assert_eq!(calibration.bias(), None);
}

#[test]
fn mag_bias_averages_symmetric_samples() {
    let mut calibration = MagBiasCalibration::<4>::new();
    let offset = Vector3::new(10.0, -5.0, 3.0);

    calibration.record(offset + Vector3::new(1.0, 0.0, 0.0), 0);
    calibration.record(offset + Vector3::new(-1.0, 0.0, 0.0), 1);
    calibration.record(offset + Vector3::new(0.0, 2.0, 0.0), 2);
    calibration.record(offset + Vector3::new(0.0, -2.0, 0.0), 3);

    assert!(calibration.is_enabled());
    assert_vec_close(calibration.bias().unwrap(), offset);
}

#[test]
fn mag_bias_rolls_over_old_samples() {
    let mut calibration = MagBiasCalibration::<3>::new();

    calibration.record(Vector3::new(1.0, 0.0, 0.0), 0);
    calibration.record(Vector3::new(2.0, 0.0, 0.0), 1);
    calibration.record(Vector3::new(3.0, 0.0, 0.0), 2);
    assert_vec_close(calibration.bias().unwrap(), Vector3::new(2.0, 0.0, 0.0));

    calibration.record(Vector3::new(4.0, 0.0, 0.0), 3);
    assert_vec_close(calibration.bias().unwrap(), Vector3::new(3.0, 0.0, 0.0));

    calibration.record(Vector3::new(5.0, 0.0, 0.0), 4);
    assert_vec_close(calibration.bias().unwrap(), Vector3::new(4.0, 0.0, 0.0));
}

#[test]
fn mag_bias_corrected_sample_subtracts_active_bias() {
    let mut calibration = MagBiasCalibration::<2>::new();
    let offset = Vector3::new(10.0, 20.0, -5.0);

    assert_vec_close(
        calibration.corrected_sample(offset + Vector3::new(3.0, 0.0, 0.0), 0),
        offset + Vector3::new(3.0, 0.0, 0.0),
    );
    assert_vec_close(
        calibration.corrected_sample(offset + Vector3::new(-3.0, 0.0, 0.0), 1),
        Vector3::new(-3.0, 0.0, 0.0),
    );
}
