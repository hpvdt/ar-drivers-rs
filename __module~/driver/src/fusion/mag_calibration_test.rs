use nalgebra::Vector3;

use super::mag_calibration::MagCalibrator;

#[test]
fn mag_calibrator_solves_synthetic_offset_and_scale() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let scale = Vector3::new(3.0, 2.0, 1.5);
    let mut calibrator = seeded_calibrator::<63>(offset, scale);

    let (actual_offset, actual_scale) = calibrator.perform_calibration().unwrap();

    assert_vec_close(Vector3::from(actual_offset), offset, 0.05);
    assert_vec_close(Vector3::from(actual_scale), scale, 0.1);
}

#[test]
fn mag_calibrator_degenerate_data_does_not_panic() {
    let mut calibrator = MagCalibrator::<6>::new();
    for _ in 0..6 {
        calibrator.evaluate_sample_vec(Vector3::new(5.0, 6.0, 7.0));
    }

    assert!(calibrator.perform_calibration().is_none());
}

fn seeded_calibrator<const N: usize>(
    offset: Vector3<f32>,
    scale: Vector3<f32>,
) -> MagCalibrator<N> {
    let mut calibrator = MagCalibrator::new();
    for i in 0..N {
        let direction = sample_direction(i, N);
        calibrator.evaluate_sample_vec(offset + scale.component_mul(&direction));
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
