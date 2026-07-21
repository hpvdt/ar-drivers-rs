use std::time::{Duration, Instant};

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
    let required_validation_duration = Duration::from_secs(5);
    let validation_duration = Duration::from_secs(20);
    let warmup_duration = Duration::from_secs(5);
    let mut first_success_at = None;
    let mut validation_start = None;
    let mut completed_required_validation = false;
    let mut worst_angle_degrees = 0.0f32;

    let test_start = Instant::now();
    loop {
        assert!(
            test_start.elapsed() <= Duration::from_secs(120),
            "magnetometer calibration never succeeded within 120 seconds"
        );
        if validation_start.is_some_and(|start: Instant| start.elapsed() >= validation_duration) {
            break;
        }
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
        let warmed_up =
            first_success_at.is_some_and(|instant: Instant| instant.elapsed() >= warmup_duration);
        if !warmed_up {
            if result.is_ok() {
                first_success_at.get_or_insert_with(Instant::now);
            }
            continue;
        }

        let corrected = result.unwrap_or_else(|error| {
            panic!("magnetometer calibration failed at timestamp={timestamp}: {error:?}")
        });
        let angle_degrees = corrected.angle(&ideal_body_frd).to_degrees();
        assert!(
            angle_degrees <= 30.0,
            "corrected magnetometer exceeded 30 degrees at timestamp={timestamp}: angle_degrees={angle_degrees}"
        );
        let start = *validation_start.get_or_insert_with(Instant::now);
        worst_angle_degrees = worst_angle_degrees.max(angle_degrees);
        if start.elapsed() >= required_validation_duration {
            completed_required_validation = true;
        }
    }

    assert!(
        completed_required_validation,
        "corrected magnetometer did not complete the required 5-second validation; worst_angle_degrees={worst_angle_degrees}"
    );
}
