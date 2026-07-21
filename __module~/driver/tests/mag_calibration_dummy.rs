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
    let seed = config.seed;
    let mut dummy = Dummy::with_config(config);
    let mut fusion = FusionState::new(Box::new(Dummy::new()));
    let required_validation_duration = Duration::from_secs(5);
    let validation_duration = Duration::from_secs(20);
    let warmup_duration = Duration::from_secs(5);
    let mut first_success_at = None;
    let mut validation_start = None;
    let mut completed_required_validation = false;
    let mut worst_angle_degrees = 0.0f32;

    let mut eval_count = 0u64;
    let mut eval_time = Duration::ZERO;
    let mut error_sum_degrees = 0.0f64;
    let mut error_count = 0u64;
    let mut until_first_success: Option<(Duration, u64)> = None;
    let mut warmup_span: Option<(Duration, u64)> = None;

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

        let eval_start = Instant::now();
        let result = fusion.mag.evaluate_correct(raw_frd, timestamp);
        eval_time += eval_start.elapsed();
        eval_count += 1;
        let angle_degrees = result
            .as_ref()
            .ok()
            .map(|corrected| corrected.angle(&ideal_body_frd).to_degrees());
        if let Some(angle_degrees) = angle_degrees {
            error_sum_degrees += f64::from(angle_degrees);
            error_count += 1;
        }

        let warmed_up =
            first_success_at.is_some_and(|instant: Instant| instant.elapsed() >= warmup_duration);
        if !warmed_up {
            if result.is_ok() && first_success_at.is_none() {
                first_success_at = Some(Instant::now());
                until_first_success = Some((test_start.elapsed(), eval_count));
            }
            continue;
        }
        if warmup_span.is_none() {
            let (_, count_at_first_success) = until_first_success.unwrap();
            warmup_span = Some((
                first_success_at.unwrap().elapsed(),
                eval_count - count_at_first_success,
            ));
        }

        if let Err(error) = result {
            panic!("magnetometer calibration failed at timestamp={timestamp}: {error:?}")
        }
        let angle_degrees = angle_degrees.unwrap();
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

    let total_time = test_start.elapsed();
    let (time_until_first_success, count_until_first_success) =
        until_first_success.expect("no successful correction");
    let (warmup_time, warmup_count) = warmup_span.expect("warm-up never completed");
    let verified_count = eval_count - count_until_first_success - warmup_count;
    let verified_time = total_time - time_until_first_success - warmup_time;
    println!("# Stats");
    println!("- PRNG seed: {seed}");
    println!("- evaluate_correct");
    println!(
        "  - avg computation time: {:.3} ms over {eval_count} calls",
        eval_time.as_secs_f64() * 1e3 / eval_count as f64,
    );
    println!(
        "  - avg error: {:.3} deg over {error_count} successful calls",
        error_sum_degrees / error_count as f64,
    );
    println!("- total: {total_time:.2?} / {eval_count} iterations");
    println!(
        "  - until first successful correction: {time_until_first_success:.2?} / {count_until_first_success} iterations"
    );
    println!("  - sampling/optimization warm-up: {warmup_time:.2?} / {warmup_count} iterations");
    println!("  - verification: {verified_time:.2?} / {verified_count} iterations");
}
