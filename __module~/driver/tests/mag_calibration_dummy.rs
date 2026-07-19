use std::collections::HashSet;
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use ar_drivers::fusion::{rub_to_frd, FusionState};
use ar_drivers::{ARGlasses, Dummy, DummyConfig, GlassesEvent};
use nalgebra::Vector3;
use rand::Rng;

static TIMING_TEST_LOCK: Mutex<()> = Mutex::new(());

#[derive(Default)]
struct EvaluationTiming {
    call_count: u32,
    total_duration: Duration,
}

impl EvaluationTiming {
    fn record(&mut self, duration: Duration) {
        self.call_count += 1;
        self.total_duration += duration;
    }

    fn merge(&mut self, other: Self) {
        self.call_count += other.call_count;
        self.total_duration += other.total_duration;
    }

    fn average_duration(&self) -> Duration {
        if self.call_count == 0 {
            Duration::from_secs(0)
        } else {
            self.total_duration / self.call_count
        }
    }
}

struct CalibrationRound {
    timing: EvaluationTiming,
    failure: Option<String>,
}

#[test]
fn dummy_magnetometer_calibration_stabilizes_and_remains_accurate_for_twenty_seconds() {
    let _guard = timing_test_guard();
    let seed = DummyConfig::default().seed;
    let round = run_calibration_round(seed);

    print_average_duration("default-seed magnetometer calibration test", &round.timing);

    if let Some(failure) = round.failure {
        panic!("magnetometer calibration failed for seed={seed}: {failure}");
    }
}

#[test]
fn dummy_magnetometer_calibration_handles_twenty_random_seeds() {
    const ROUND_COUNT: usize = 20;

    let _guard = timing_test_guard();
    let default_seed = DummyConfig::default().seed;
    let mut rng = rand::thread_rng();
    let mut sampled_seeds = HashSet::with_capacity(ROUND_COUNT);
    let mut timing = EvaluationTiming::default();
    let mut failed_round = None;

    for round_number in 1..=ROUND_COUNT {
        let seed = loop {
            let candidate = rng.gen::<u64>();
            if candidate != default_seed && sampled_seeds.insert(candidate) {
                break candidate;
            }
        };
        let round = run_calibration_round(seed);
        timing.merge(round.timing);

        if let Some(failure) = round.failure {
            println!(
                "randomized magnetometer calibration failed: round={round_number}, seed={seed}, failure={failure}"
            );
            failed_round = Some((round_number, seed, failure));
            break;
        }
    }

    print_average_duration("randomized magnetometer calibration test", &timing);

    if let Some((round_number, seed, failure)) = failed_round {
        panic!(
            "randomized magnetometer calibration failed at round={round_number}, seed={seed}: {failure}"
        );
    }
}

fn run_calibration_round(seed: u64) -> CalibrationRound {
    let config = DummyConfig {
        seed,
        ..DummyConfig::default()
    };
    let dip = config
        .magnetic_dip_rad
        .clamp(-30.0f32.to_radians(), 30.0f32.to_radians());
    let magnetic_world_rub =
        Vector3::new(0.0, dip.sin(), -dip.cos()) * config.magnetic_field_strength;
    let mut dummy = Dummy::with_config(config);
    let mut fusion = FusionState::new(Box::new(Dummy::new()));
    let required_window = Duration::from_secs(5);
    let test_duration = Duration::from_secs(20);
    let calibration_warmup_sample_count = 700;
    let mut timing = EvaluationTiming::default();
    let mut magnetometer_sample_count = 0;
    let mut window_start = None;
    let mut completed_validation_window = false;
    let mut worst_angle_degrees = 0.0f32;
    let deadline = Instant::now() + test_duration;

    while Instant::now() < deadline {
        let ground_truth = dummy.snapshot();
        let event = match dummy.read_event() {
            Ok(event) => event,
            Err(error) => {
                return failed_round(timing, format!("dummy event read failed: {error:?}"));
            }
        };
        let GlassesEvent::Magnetometer {
            magnetometer,
            timestamp,
        } = event
        else {
            continue;
        };

        if timestamp != ground_truth.timestamp_us {
            return failed_round(
                timing,
                format!(
                    "event timestamp did not match ground truth: timestamp={timestamp}, ground_truth_timestamp={}",
                    ground_truth.timestamp_us
                ),
            );
        }
        let ideal_body_rub = ground_truth.attitude.inverse() * magnetic_world_rub;
        let ideal_body_frd = rub_to_frd(&ideal_body_rub).normalize();
        let raw_frd = rub_to_frd(&magnetometer);

        let evaluation_start = Instant::now();
        let result = fusion.mag.evaluate_correct(raw_frd, timestamp);
        timing.record(evaluation_start.elapsed());
        magnetometer_sample_count += 1;
        if magnetometer_sample_count < calibration_warmup_sample_count {
            continue;
        }

        let corrected = match result {
            Ok(corrected) => corrected,
            Err(error) => {
                return failed_round(
                    timing,
                    format!("magnetometer calibration failed at timestamp={timestamp}: {error:?}"),
                );
            }
        };
        let angle_degrees = corrected.angle(&ideal_body_frd).to_degrees();
        if angle_degrees > 20.0 {
            return failed_round(
                timing,
                format!(
                    "corrected magnetometer exceeded 20 degrees at timestamp={timestamp}: angle_degrees={angle_degrees}"
                ),
            );
        }
        let start = *window_start.get_or_insert_with(Instant::now);
        worst_angle_degrees = worst_angle_degrees.max(angle_degrees);
        if start.elapsed() >= required_window {
            completed_validation_window = true;
        }
    }

    let failure = (!completed_validation_window).then(|| {
        format!(
            "corrected magnetometer did not complete a 5-second validation window within the 20-second wall-time limit; magnetometer_sample_count={magnetometer_sample_count}, worst_angle_degrees={worst_angle_degrees}"
        )
    });

    CalibrationRound { timing, failure }
}

fn failed_round(timing: EvaluationTiming, failure: String) -> CalibrationRound {
    CalibrationRound {
        timing,
        failure: Some(failure),
    }
}

fn print_average_duration(test_name: &str, timing: &EvaluationTiming) {
    println!(
        "{test_name}: evaluate_correct_call_count={}, average_evaluate_correct_duration={:?}",
        timing.call_count,
        timing.average_duration()
    );
}

fn timing_test_guard() -> MutexGuard<'static, ()> {
    TIMING_TEST_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}
