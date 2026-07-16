use ar_drivers::fusion::{rub_to_frd, FusionState};
use ar_drivers::{ARGlasses, Dummy, DummyConfig, GlassesEvent};
use nalgebra::Vector3;

#[test]
fn corrected_dummy_magnetometer_stays_within_twenty_degrees_for_five_seconds() {
    let config = DummyConfig::default();
    let dip = config
        .magnetic_dip_rad
        .clamp(-30.0f32.to_radians(), 30.0f32.to_radians());
    let magnetic_world_rub =
        Vector3::new(0.0, dip.sin(), -dip.cos()) * config.magnetic_field_strength;
    let mut dummy = Dummy::with_config(config);
    let mut fusion = FusionState::new(Box::new(Dummy::new()));
    let required_window_us = 5_000_000;
    let simulation_limit_us = 20_000_000;
    let calibration_sample_count = 255;
    let mut accepted_samples = 0;
    let mut calibration = None;
    let mut window_start_us = None;
    let mut worst_angle_degrees = 0.0f32;

    while dummy.snapshot().timestamp_us <= simulation_limit_us {
        let ground_truth = dummy.snapshot();
        let event = dummy.read_event().unwrap();
        let GlassesEvent::Magnetometer {
            magnetometer,
            timestamp,
        } = event
        else {
            continue;
        };

        assert_eq!(timestamp, ground_truth.timestamp_us);
        let ideal_body_rub = ground_truth.attitude.inverse() * magnetic_world_rub;
        let ideal_body_frd = rub_to_frd(&ideal_body_rub).normalize();
        let raw_frd = rub_to_frd(&magnetometer);
        if calibration.is_none() {
            fusion.mag.evaluate_sample_vec(raw_frd, timestamp);
            accepted_samples += 1;
            if accepted_samples == calibration_sample_count {
                calibration = Some(fusion.mag.perform_calibration().unwrap());
            }
            continue;
        }

        let (offset, cholesky) = calibration.as_ref().unwrap();
        let correction = cholesky * cholesky.transpose();
        let corrected = (correction * (raw_frd - offset)).normalize();
        let angle_degrees = corrected.angle(&ideal_body_frd).to_degrees();
        assert!(
            angle_degrees <= 20.0,
            "corrected magnetometer exceeded 20 degrees at timestamp={timestamp}: angle_degrees={angle_degrees}"
        );
        let start = *window_start_us.get_or_insert(timestamp);
        worst_angle_degrees = worst_angle_degrees.max(angle_degrees);
        if timestamp - start >= required_window_us {
            return;
        }
    }

    panic!(
        "corrected magnetometer did not complete a 5-second validation window; worst_angle_degrees={worst_angle_degrees}"
    );
}
