use nalgebra::{UnitQuaternion, Vector3};

use super::mag_calibration::MagCalibrator;
use super::naive_cf::NaiveCF;
use super::{mag_calibration_can_divide, FusionState, MIN_MAG_SCALE_DIVISOR};

fn frd_to_rub(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(v.y, -v.z, -v.x)
}

#[test]
fn update_mag_uses_shared_mag_calibrator() {
    let mut fusion = NaiveCF::new(Box::new(crate::dummy::Dummy {})).unwrap();
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let scale = Vector3::new(3.0, 2.0, 1.5);
    fusion.state.mag = seeded_calibrator(offset, scale);
    fusion.state.attitude = UnitQuaternion::identity();
    fusion.state.corrections.mag = Default::default();

    let calibrated_north = Vector3::new(1.0, 0.0, 0.0);
    let raw_north = offset + scale.component_mul(&calibrated_north);
    let north_rub = frd_to_rub(raw_north);

    fusion.update_mag(&north_rub, 0);

    assert!(fusion.state.corrections.mag.prev < 0.001);
    assert!(fusion.state.attitude.angle() < 0.001);
}

#[test]
fn get_calibrated_mag_discards_unsafe_scale_divisor() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let scale = Vector3::new(3.0, MIN_MAG_SCALE_DIVISOR * 0.5, 1.5);

    assert!(!mag_calibration_can_divide(&offset, &scale));
}

#[test]
fn get_calibrated_mag_discards_unavailable_calibration() {
    let mut state = FusionState::new(Box::new(crate::dummy::Dummy {}));
    let raw_mag = Vector3::new(5.0, 6.0, 7.0);

    assert!(matches!(
        state.getCalibratedMag(raw_mag),
        Err(super::BadMagDataCause::InsufficientSamples)
    ));
}

// #[test]
// fn get_calibrated_mag_discards_weak_raw_reading() {
//     let mut state = FusionState::new(Box::new(crate::dummy::Dummy {}));
//     let raw_mag = Vector3::new(0.1, 0.1, 0.1);
//
//     assert!(matches!(
//         state.getCalibratedMag(raw_mag),
//         Err(super::BadMagDataCause::WeakRawReading { .. })
//     ));
// }

fn seeded_calibrator(offset: Vector3<f32>, scale: Vector3<f32>) -> MagCalibrator<63> {
    let mut calibrator = MagCalibrator::new();
    for i in 0..63 {
        let direction = sample_direction(i);
        calibrator.evaluate_sample_vec(offset + scale.component_mul(&direction));
    }
    calibrator
}

fn sample_direction(i: usize) -> Vector3<f32> {
    let theta = 0.37 + i as f32 * 1.21;
    let z = -0.8 + 1.6 * i as f32 / 62.0;
    let radius = (1.0 - z * z).sqrt();
    Vector3::new(radius * theta.cos(), radius * theta.sin(), z)
}
