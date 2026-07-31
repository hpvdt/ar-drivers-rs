use std::time::{Duration, Instant};

use ar_drivers::fusion::{rub_to_frd, FusionState};
use ar_drivers::{ARGlasses, Dummy, DummyConfig, GlassesEvent};
use nalgebra::Vector3;
use serial_test::serial;

/// Whether the calibrator is fed a co-timestamped simulated accelerometer reading with each sample.
#[derive(Clone, Copy)]
enum AttitudeMode {
    Always,
    Never,
}

struct RunStats {
    eval_time: Duration,
    eval_count: u64,
    error_sum_degrees: f64,
    error_count: u64,
    worst_error_degrees: f32,
    validation_error_sum_degrees: f64,
    validation_error_count: u64,
    worst_validation_error_degrees: f32,
    total_time: Duration,
    time_until_first_success: Duration,
    count_until_first_success: u64,
    warmup_time: Duration,
    warmup_count: u64,
    verified_time: Duration,
    verified_count: u64,
}

fn run_calibration(config: DummyConfig, attitude_mode: AttitudeMode) -> RunStats {
    let seed = config.seed;
    let mode_label = match attitude_mode {
        AttitudeMode::Always => "with accelerometer gravity",
        AttitudeMode::Never => "without gravity",
    };
    println!("# Starting benchmark - PRNG seed: {seed}, {mode_label}");

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
    let mut worst_error_degrees = 0.0f32;

    let mut eval_count = 0u64;
    let mut eval_time = Duration::ZERO;
    let mut error_sum_degrees = 0.0f64;
    let mut error_count = 0u64;
    let mut validation_error_sum_degrees = 0.0f64;
    let mut validation_error_count = 0u64;
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
        let accelerometer_frd = (!ground_truth.next_event_is_acc_gyro)
            .then(|| rub_to_frd(&dummy.accelerometer_reading()));
        let event = dummy.read_event().unwrap();
        let (magnetometer, timestamp) = match event {
            GlassesEvent::AccGyro { .. } => continue,
            GlassesEvent::Magnetometer {
                magnetometer,
                timestamp,
            } => (magnetometer, timestamp),
            _ => continue,
        };

        assert_eq!(timestamp, ground_truth.timestamp_us);
        let ideal_body_rub = ground_truth.attitude.inverse() * magnetic_world_rub;
        let ideal_body_frd = rub_to_frd(&ideal_body_rub).normalize();
        let raw_frd = rub_to_frd(&magnetometer);

        let eval_start = Instant::now();
        let gravity_direction = match attitude_mode {
            AttitudeMode::Always => Some(
                accelerometer_frd
                    .expect("dummy magnetometer event did not have an accelerometer sample"),
            ),
            AttitudeMode::Never => None,
        };
        let result = fusion
            .mag
            .evaluate_correct(raw_frd, gravity_direction, timestamp);
        eval_time += eval_start.elapsed();
        eval_count += 1;
        let angle_degrees = result
            .as_ref()
            .ok()
            .map(|corrected| corrected.angle(&ideal_body_frd).to_degrees());
        if let Some(angle_degrees) = angle_degrees {
            error_sum_degrees += f64::from(angle_degrees);
            error_count += 1;
            worst_error_degrees = worst_error_degrees.max(angle_degrees);
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
            angle_degrees <= 18.0,
            "corrected magnetometer exceeded 18 degrees at timestamp={timestamp}: angle_degrees={angle_degrees}"
        );
        validation_error_sum_degrees += f64::from(angle_degrees);
        validation_error_count += 1;
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

    println!("- evaluate_correct");
    println!(
        "  - avg computation time: {:.3} ms over {eval_count} calls",
        eval_time.as_secs_f64() * 1e3 / eval_count as f64,
    );
    println!(
        "  - avg error: {:.3} deg over {error_count} successful calls",
        error_sum_degrees / error_count as f64,
    );
    println!("  - worst error: {worst_error_degrees:.3} deg");
    println!(
        "  - avg post-warmup error: {:.3} deg over {validation_error_count} calls",
        validation_error_sum_degrees / validation_error_count.max(1) as f64,
    );
    println!("  - worst post-warmup error: {worst_angle_degrees:.3} deg");
    println!("- total: {total_time:.2?} / {eval_count} iterations");
    println!(
        "  - until first successful correction: {time_until_first_success:.2?} / {count_until_first_success} iterations"
    );
    println!("  - sampling/optimization warm-up: {warmup_time:.2?} / {warmup_count} iterations");
    println!("  - verification: {verified_time:.2?} / {verified_count} iterations");
    RunStats {
        eval_time,
        eval_count,
        error_sum_degrees,
        error_count,
        worst_error_degrees,
        validation_error_sum_degrees,
        validation_error_count,
        worst_validation_error_degrees: worst_angle_degrees,
        total_time,
        time_until_first_success,
        count_until_first_success,
        warmup_time,
        warmup_count,
        verified_time,
        verified_count,
    }
}

fn print_avg_stats(runs: &[RunStats]) {
    let n = runs.len() as f64;
    let avg_dur = |f: fn(&RunStats) -> Duration| {
        Duration::from_secs_f64(runs.iter().map(|r| f(r).as_secs_f64()).sum::<f64>() / n)
    };
    let avg_count =
        |f: fn(&RunStats) -> u64| (runs.iter().map(|r| f(r)).sum::<u64>() as f64 / n).round();
    let total_eval_time: f64 = runs.iter().map(|r| r.eval_time.as_secs_f64()).sum();
    let total_eval_count: u64 = runs.iter().map(|r| r.eval_count).sum();
    let total_error_sum: f64 = runs.iter().map(|r| r.error_sum_degrees).sum();
    let total_error_count: u64 = runs.iter().map(|r| r.error_count).sum();
    let total_validation_error_sum: f64 = runs.iter().map(|r| r.validation_error_sum_degrees).sum();
    let total_validation_error_count: u64 = runs.iter().map(|r| r.validation_error_count).sum();
    let worst_error_degrees: f32 = runs
        .iter()
        .map(|r| r.worst_error_degrees)
        .fold(0.0, f32::max);
    let worst_validation_error_degrees: f32 = runs
        .iter()
        .map(|r| r.worst_validation_error_degrees)
        .fold(0.0, f32::max);

    println!("  ======================================================================  ");
    println!("# Average stats over {} runs", runs.len());
    println!("- evaluate_correct");
    println!(
        "  - avg computation time: {:.3} ms over {} calls",
        total_eval_time * 1e3 / total_eval_count as f64,
        avg_count(|r| r.eval_count),
    );
    println!(
        "  - avg error: {:.3} deg over {} successful calls",
        total_error_sum / total_error_count as f64,
        avg_count(|r| r.error_count),
    );
    println!("  - worst error: {worst_error_degrees:.3} deg");
    println!(
        "  - avg post-warmup error: {:.3} deg over {} calls",
        total_validation_error_sum / total_validation_error_count as f64,
        avg_count(|r| r.validation_error_count),
    );
    println!("  - worst post-warmup error: {worst_validation_error_degrees:.3} deg");
    println!(
        "- total: {:.2?} / {} iterations",
        avg_dur(|r| r.total_time),
        avg_count(|r| r.eval_count),
    );
    println!(
        "  - until first successful correction: {:.2?} / {} iterations",
        avg_dur(|r| r.time_until_first_success),
        avg_count(|r| r.count_until_first_success),
    );
    println!(
        "  - sampling/optimization warm-up: {:.2?} / {} iterations",
        avg_dur(|r| r.warmup_time),
        avg_count(|r| r.warmup_count),
    );
    println!(
        "  - verification: {:.2?} / {} iterations",
        avg_dur(|r| r.verified_time),
        avg_count(|r| r.verified_count),
    );
}

/// Runs each seed with the given attitude mode, printing average stats.
fn run_seeds(attitude_mode: AttitudeMode, seeds: impl IntoIterator<Item = u64>) {
    let runs: Vec<RunStats> = seeds
        .into_iter()
        .map(|seed| {
            let config = DummyConfig {
                seed,
                ..DummyConfig::default()
            };
            run_calibration(config, attitude_mode)
        })
        .collect();
    print_avg_stats(&runs);
}

#[test_case::test_case(AttitudeMode::Never  ; "without_gravity")]
#[test_case::test_case(AttitudeMode::Always ; "with_gravity")]
#[serial]
fn short(attitude_mode: AttitudeMode) {
    run_seeds(attitude_mode, [rand::random()]);
}

#[test_case::test_case(AttitudeMode::Never  ; "without_gravity")]
#[test_case::test_case(AttitudeMode::Always ; "with_gravity")]
#[serial]
fn long(attitude_mode: AttitudeMode) {
    run_seeds(attitude_mode, (0..10).map(|_| rand::random()));
}

#[test_case::test_case(AttitudeMode::Never  ; "without_gravity")]
#[test_case::test_case(AttitudeMode::Always ; "with_gravity")]
#[serial]
fn regression(attitude_mode: AttitudeMode) {
    run_seeds(
        attitude_mode,
        [
            934786981548549007,
            320366629120039532,
            800448092538851856,
            14346460742415463748,
        ],
    );
}
