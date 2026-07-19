use ar_drivers::fusion::{rub_to_frd, FusionState};
use ar_drivers::{ARGlasses, Dummy, DummyConfig, GlassesEvent};
use nalgebra::Vector3;

#[test]
fn dummy_magnetometer_calibration_stabilizes_and_remains_accurate_for_twenty_seconds() {
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
    let calibration_warmup_sample_count = 510;
    let mut magnetometer_sample_count = 0;
    let mut window_start_us = None;
    let mut completed_validation_window = false;
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

        let result = fusion.mag.evaluate_correct(raw_frd, timestamp);
        magnetometer_sample_count += 1;
        if magnetometer_sample_count < calibration_warmup_sample_count {
            continue;
        }

        let corrected = result.unwrap_or_else(|error| {
            panic!("magnetometer calibration failed at timestamp={timestamp}: {error:?}")
        });
        let angle_degrees = corrected.angle(&ideal_body_frd).to_degrees();
        assert!(
            angle_degrees <= 20.0,
            "corrected magnetometer exceeded 20 degrees at timestamp={timestamp}: angle_degrees={angle_degrees}"
        );
        let start = *window_start_us.get_or_insert(timestamp);
        worst_angle_degrees = worst_angle_degrees.max(angle_degrees);
        if timestamp - start >= required_window_us {
            completed_validation_window = true;
        }
    }

    assert!(
        completed_validation_window,
        "corrected magnetometer did not complete a 5-second validation window; worst_angle_degrees={worst_angle_degrees}"
    );
}
