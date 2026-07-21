use nalgebra::{UnitQuaternion, Vector3};

use super::bad_mag_cause::{BadCalibration, BadMagCause};
use super::mag_calibration::MagCalibrator;
use super::naive_cf::NaiveCF;
use super::FusionState;

fn frd_to_rub(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(v.y, -v.z, -v.x)
}

#[test]
fn update_mag_uses_shared_mag_calibrator() {
    let mut fusion = NaiveCF::new(Box::new(crate::sim::Dummy::new())).unwrap();
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
fn get_calibrated_mag_discards_underconstrained_calibration() {
    let mut state = FusionState::new(Box::new(crate::sim::Dummy::new()));
    let raw_mag = Vector3::new(5.0, 6.0, 7.0);

    assert!(matches!(
        state.mag.evaluate_correct(raw_mag, 0),
        Err(BadMagCause::BadCalibration(
            BadCalibration::InsufficientSamples {
                samples: 1,
                required: 255
            }
        ))
    ));
}

#[test]
fn update_mag_discards_ill_conditioned_calibration() {
    let mut fusion = NaiveCF::new(Box::new(crate::sim::Dummy::new())).unwrap();
    fusion.state.mag = nearly_collinear_calibrator();
    fusion.state.attitude = UnitQuaternion::identity();
    fusion.state.corrections.mag = Default::default();

    let raw_mag = Vector3::new(10.0005, -4.9990, 3.00025);
    let mag_rub = frd_to_rub(raw_mag);

    fusion.update_mag(&mag_rub, 0);

    assert_eq!(fusion.state.corrections.mag.prev, 0.0);
    assert_eq!(fusion.state.corrections.mag.avg, 0.0);
    assert_eq!(fusion.state.attitude.angle(), 0.0);
}

// #[test]
// fn get_calibrated_mag_discards_weak_raw_reading() {
//     let mut state = FusionState::new(Box::new(crate::sim::Dummy {}));
//     let raw_mag = Vector3::new(0.1, 0.1, 0.1);
//
//     assert!(matches!(
//         state.getCalibratedMag(raw_mag),
//         Err(super::BadMagCause::BadReading(super::BadReading::WeakRawReading { .. }))
//     ));
// }

fn seeded_calibrator(offset: Vector3<f32>, scale: Vector3<f32>) -> MagCalibrator<255> {
    let mut calibrator = MagCalibrator::new();
    for i in 0..255 {
        let direction = sample_direction(i);
        let _ = calibrator.evaluate_correct(offset + scale.component_mul(&direction), i as u64);
    }
    calibrator
}

fn nearly_collinear_calibrator() -> MagCalibrator<255> {
    let mut calibrator = MagCalibrator::new();
    for i in 0..255 {
        let t = i as f32 * 0.0001;
        let _ = calibrator.evaluate_correct(
            Vector3::new(10.0 + t, -5.0 + 2.0 * t, 3.0 + 0.5 * t),
            i as u64,
        );
    }
    calibrator
}

fn sample_direction(i: usize) -> Vector3<f32> {
    let theta = 0.37 + i as f32 * 1.21;
    let z = -0.8 + 1.6 * i as f32 / 254.0;
    let radius = (1.0 - z * z).sqrt();
    Vector3::new(radius * theta.cos(), radius * theta.sin(), z)
}
