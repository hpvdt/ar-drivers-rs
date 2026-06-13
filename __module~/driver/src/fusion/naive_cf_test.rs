use nalgebra::{UnitQuaternion, Vector3};

use super::mag_bias_calibration::MagBiasCalibration;
use super::naive_cf::NaiveCF;

fn frd_to_rub(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(v.y, -v.z, -v.x)
}

#[test]
fn update_mag_rejects_near_zero_corrected_field() {
    let mut fusion = NaiveCF::new(Box::new(crate::dummy::Dummy {})).unwrap();
    let north_frd = Vector3::new(1.0, 0.0, 0.0);
    let north_rub = frd_to_rub(north_frd);

    fusion.update_mag(&north_rub, 0);
    fusion.state.attitude = UnitQuaternion::identity();
    fusion.state.corrections.mag = Default::default();

    fusion.update_mag(&north_rub, MagBiasCalibration::<5000>::ENABLE_AFTER_US);

    assert_eq!(fusion.state.corrections.mag.prev, 0.0);
    assert!(fusion.state.attitude.angle() < 0.0001);
}
