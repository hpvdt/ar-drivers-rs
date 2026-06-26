use super::mag_calibration::MagCalibrator;

#[test]
fn mag_calibration_rejects_rank_deficient_samples() {
    let mut calibration = MagCalibrator::<8>::new();

    for _ in 0..8 {
        calibration.evaluate_sample([37.0, 38.0, 39.0]);
    }

    assert_eq!(calibration.perform_calibration(), None);
}
