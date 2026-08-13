use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use super::bad_mag_cause::BadMagCause;
use super::mag_calibration::{
    MagCalibrationResult, MagCalibrator, MIN_PUBLICATION_CONFIDENCE, MIN_PUBLICATION_STREAK,
};

#[test]
fn mag_calibrator_corrects_synthetic_full_spd_distortion() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<63>(offset, distortion);

    for (timestamp_us, expected) in [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, -2.0, 3.0).normalize(),
    ]
    .into_iter()
    .enumerate()
    {
        let corrected = calibrated(calibrator.evaluate_correct(
            offset + distortion * expected,
            None,
            timestamp_us as u64,
        ));
        assert_vec_close(corrected, expected, 0.05);
    }
}

#[test]
fn mag_calibrator_corrects_asymmetrically_sampled_distortion() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = MagCalibrator::<63>::new();
    for i in 0..63 {
        let theta = 0.37 + i as f32 * 1.21;
        let z = -0.1 + i as f32 / 62.0;
        let radius = (1.0 - z * z).sqrt();
        let direction = Vector3::new(radius * theta.cos(), radius * theta.sin(), z);
        let _ = calibrator.evaluate_correct(offset + distortion * direction, None, i as u64);
    }

    for i in 0..16 * 63 {
        let sample_index = i % 63;
        let theta = 0.37 + sample_index as f32 * 1.21;
        let z = -0.1 + sample_index as f32 / 62.0;
        let radius = (1.0 - z * z).sqrt();
        let direction = Vector3::new(radius * theta.cos(), radius * theta.sin(), z);
        let _ = calibrator.evaluate_correct(offset + distortion * direction, None, (64 + i) as u64);
    }
    let expected = Vector3::new(1.0, -2.0, -1.0).normalize();
    let corrected =
        calibrated(calibrator.evaluate_correct(offset + distortion * expected, None, 64 + 16 * 63));

    assert_vec_close(corrected, expected, 0.05);
}

#[test]
fn mag_calibrator_returns_stable_online_corrections() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<63>(offset, distortion);
    let expected = Vector3::new(1.0, -2.0, 3.0).normalize();
    let raw = offset + distortion * expected;

    let first = calibrated(calibrator.evaluate_correct(raw, None, 1));
    let second = calibrated(calibrator.evaluate_correct(raw, None, 2));

    assert_vec_close(second, first, 0.01);
}

#[test]
fn mag_calibrator_stays_pending_with_underconstrained_or_degenerate_data() {
    let mut calibrator = MagCalibrator::<9>::new();
    let sample = Vector3::new(5.0, 6.0, 7.0);
    let single = calibrator.evaluate_correct(sample, None, 0);

    assert!(matches!(
        single,
        Ok(MagCalibrationResult::Pending { confidence: 0.0 })
    ));
    let result = (1..9)
        .map(|timestamp_us| calibrator.evaluate_correct(sample, None, timestamp_us))
        .last()
        .unwrap();

    assert!(matches!(
        result,
        Ok(MagCalibrationResult::Pending { confidence: 0.0 })
    ));
    assert_eq!(calibrator.get_confidence(), 0.0);
}

#[test]
fn mag_calibrator_publishes_before_the_buffer_is_full() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = MagCalibrator::<1023>::new();
    let mut published_at = None;
    let mut qualifying_streak = 0;
    for i in 0..1022 {
        let direction = sample_direction(i % 63, 63);
        let result = calibrator
            .evaluate_correct(offset + distortion * direction, None, i as u64)
            .unwrap();
        if result.confidence() >= MIN_PUBLICATION_CONFIDENCE {
            qualifying_streak += 1;
        } else {
            qualifying_streak = 0;
        }
        if result.calibrated().is_some() {
            assert!(qualifying_streak >= MIN_PUBLICATION_STREAK);
            published_at = Some(i + 1);
            break;
        }
    }

    let components = calibrator.working_quality_components();
    let published_at = published_at.unwrap_or_else(|| {
        panic!("partial cache never published a valid correction: components={components:?}")
    });
    assert!(published_at >= 9);
    assert!(published_at < 1023);
    assert!(calibrator.get_confidence() >= MIN_PUBLICATION_CONFIDENCE);
}

#[test]
fn mag_calibrator_resets_publication_streak_after_invalid_sample() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = MagCalibrator::<1023>::new();

    for i in 0..1022 {
        let direction = sample_direction(i % 63, 63);
        let _ = calibrator.evaluate_correct(offset + distortion * direction, None, i as u64);
        if calibrator.publication_quality_streak_for_test() > 0 {
            let _ = calibrator.evaluate_correct(Vector3::repeat(f32::NAN), None, i as u64 + 1);
            assert_eq!(calibrator.publication_quality_streak_for_test(), 0);
            return;
        }
    }

    panic!("confidence never began a publication-quality streak");
}

#[test]
fn mag_calibrator_rejects_nearly_collinear_samples() {
    let mut calibrator = MagCalibrator::<12>::new();
    let mut result = None;
    for i in 0..12 {
        let t = i as f32 * 0.0001;
        result = Some(calibrator.evaluate_correct(
            Vector3::new(10.0 + t, -5.0 + 2.0 * t, 3.0 + 0.5 * t),
            None,
            i as u64,
        ));
    }

    assert!(matches!(
        result.unwrap(),
        Ok(MagCalibrationResult::Pending { confidence: 0.0 })
    ));
}

#[test]
fn mag_calibrator_keeps_last_correction_after_rejected_refit() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<63>(offset, distortion).max_sample_lifespan_us(0);
    let mut result = None;
    let expected = Vector3::x();
    let raw = offset + distortion * expected;

    for _ in 0..63 {
        result = Some(calibrator.evaluate_correct(raw, None, 1));
    }

    let result = result.unwrap().unwrap();
    assert_eq!(result.confidence(), 0.0);
    assert_vec_close(
        result
            .calibrated()
            .expect("last published correction was discarded"),
        expected,
        0.05,
    );
    assert_eq!(calibrator.radial_residual_mean_square_for_test(), None);
}

#[test]
fn mag_calibrator_clamps_neighbor_count_through_public_result() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let expected = Vector3::new(1.0, -2.0, 3.0).normalize();

    for k in [0, 99] {
        let mut calibrator = seeded_calibrator::<63>(offset, distortion).num_neighbors(k);
        let corrected =
            calibrated(calibrator.evaluate_correct(offset + distortion * expected, None, 1));
        assert_vec_close(corrected, expected, 0.05);
    }
}

#[test]
fn mag_calibrator_clamps_minibatch_size_and_is_deterministic() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let expected = Vector3::new(1.0, -2.0, 3.0).normalize();
    let raw = offset + distortion * expected;

    for minibatch_size in [0, usize::MAX] {
        let mut first = MagCalibrator::<63>::new().minibatch_size(minibatch_size);
        let mut second = MagCalibrator::<63>::new().minibatch_size(minibatch_size);
        for i in 0..17 * 63 {
            let direction = sample_direction(i % 63, 63);
            let raw = offset + distortion * direction;
            assert_eq!(
                first.evaluate_correct(raw, None, i as u64),
                second.evaluate_correct(raw, None, i as u64)
            );
        }
        let first_result = first.evaluate_correct(raw, None, 10_000);
        let second_result = second.evaluate_correct(raw, None, 10_000);
        assert_eq!(first_result, second_result);
        if minibatch_size == usize::MAX {
            assert_vec_close(calibrated(first_result), expected, 0.05);
        }
    }
}

#[test]
fn mag_calibrator_converges_faster_with_cache_replay() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut replayed = MagCalibrator::<63>::new();
    let mut plain = MagCalibrator::<63>::new().replay_updates(0);

    let probe_error = |calibrator: &MagCalibrator<63>| {
        [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Vector3::new(1.0, -2.0, 3.0).normalize(),
        ]
        .into_iter()
        .try_fold(0.0, |sum, expected| {
            calibrator
                .correct_working_for_test(offset + distortion * expected)
                .map(|actual| sum + (actual - expected).norm())
        })
    };

    let mut replayed_published_at = None;
    let mut plain_published_at = None;
    let mut replayed_converged_at = None;
    let mut plain_converged_at = None;
    for i in 0..16 * 63 {
        let raw = offset + distortion * sample_direction(i % 63, 63);
        let replayed_result = replayed.evaluate_correct(raw, None, i as u64).unwrap();
        let plain_result = plain.evaluate_correct(raw, None, i as u64).unwrap();
        if replayed_result.calibrated().is_some() && replayed_published_at.is_none() {
            replayed_published_at = Some(i);
        }
        if plain_result.calibrated().is_some() && plain_published_at.is_none() {
            plain_published_at = Some(i);
        }
        if replayed_converged_at.is_none()
            && probe_error(&replayed).is_some_and(|error| error < 0.02)
        {
            replayed_converged_at = Some(i);
        }
        if plain_converged_at.is_none() && probe_error(&plain).is_some_and(|error| error < 0.02) {
            plain_converged_at = Some(i);
        }
    }
    let replayed_published_at = replayed_published_at.expect("replayed calibration stayed pending");
    let plain_published_at = plain_published_at.expect("plain calibration stayed pending");
    let replayed_converged_at =
        replayed_converged_at.expect("replayed calibration did not converge");
    let plain_converged_at = plain_converged_at.expect("plain calibration did not converge");
    assert!(
        replayed_converged_at < plain_converged_at,
        "replayed_converged_at={replayed_converged_at} plain_converged_at={plain_converged_at}"
    );
    assert!(replayed_published_at <= plain_published_at);

    let replayed_error = probe_error(&replayed).unwrap();
    let plain_error = probe_error(&plain).unwrap();

    assert!(replayed_error < 0.02, "replayed_error={replayed_error}");
    assert!(plain_error < 0.02, "plain_error={plain_error}");
}

#[test]
fn mag_calibrator_clamps_replay_configuration_and_is_deterministic() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let expected = Vector3::new(1.0, -2.0, 3.0).normalize();
    let raw = offset + distortion * expected;

    for (replay_updates, replay_minibatch_size) in [(0, 0), (3, 0), (3, usize::MAX)] {
        let configure = || {
            MagCalibrator::<63>::new()
                .replay_updates(replay_updates)
                .replay_minibatch_size(replay_minibatch_size)
        };
        let mut first = configure();
        let mut second = configure();
        for i in 0..17 * 63 {
            let direction = sample_direction(i % 63, 63);
            let raw = offset + distortion * direction;
            assert_eq!(
                first.evaluate_correct(raw, None, i as u64),
                second.evaluate_correct(raw, None, i as u64)
            );
        }
        let first_result = first.evaluate_correct(raw, None, 10_000);
        let second_result = second.evaluate_correct(raw, None, 10_000);
        assert_eq!(first_result, second_result);
        // A single-observation replay minibatch is degenerate, like a
        // single-observation anchored minibatch, so only configurations with
        // enough observations per update are required to converge.
        if replay_updates == 0 || replay_minibatch_size == usize::MAX {
            assert_vec_close(calibrated(first_result), expected, 0.05);
        }
    }
}

#[test]
fn mag_calibrator_accepts_zero_components_and_rejects_bad_vectors() {
    let mut calibrator = MagCalibrator::<12>::new();
    for (timestamp_us, sample) in [
        Vector3::new(45.0, 0.0, -12.0),
        Vector3::new(f32::NAN, 1.0, 1.0),
        Vector3::new(f32::INFINITY, 1.0, 1.0),
        Vector3::new(1.0e-8, 0.0, 0.0),
    ]
    .into_iter()
    .enumerate()
    {
        let result = calibrator.evaluate_correct(sample, None, timestamp_us as u64);
        assert!(matches!(
            result,
            Ok(MagCalibrationResult::Pending { confidence: 0.0 })
        ));
    }
}

#[test]
fn mag_calibrator_defaults_sample_lifespan_to_one_hour() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<12>(offset, distortion);

    let result = calibrator.evaluate_correct(
        Vector3::new(20.0, 30.0, 40.0),
        None,
        60 * 60 * 1_000_000 + 1,
    );

    let result = result.unwrap();
    assert!(matches!(
        result,
        MagCalibrationResult::Calibrated {
            confidence: 0.0,
            ..
        }
    ));
}

#[test]
fn mag_calibrator_uses_configured_sample_lifespan() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<12>(offset, distortion).max_sample_lifespan_us(10);

    let result = calibrator.evaluate_correct(Vector3::new(20.0, 30.0, 40.0), None, 11);

    let result = result.unwrap();
    assert!(matches!(
        result,
        MagCalibrationResult::Calibrated {
            confidence: 0.0,
            ..
        }
    ));
}

#[test]
fn mag_calibrator_improves_with_consistent_gravity() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let world_mag = Vector3::new(0.8, 0.1, 0.5).normalize();
    let world_gravity = Vector3::z();
    let mut plain = MagCalibrator::<63>::new();
    let mut gravity_refined = MagCalibrator::<63>::new();

    for i in 0..63 {
        let attitude = UnitQuaternion::from_euler_angles(
            0.25 * (i as f32 * 0.7).sin(),
            0.35 * (i as f32 * 1.7).sin(),
            i as f32 * 2.4,
        );
        let body_mag = attitude.inverse() * world_mag;
        let body_gravity = attitude.inverse() * world_gravity;
        let raw = offset + distortion * body_mag;
        let _ = plain.evaluate_correct(raw, None, i as u64);
        let _ = gravity_refined.evaluate_correct(raw, Some(body_gravity), i as u64);
    }

    let probes = [
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::new(1.0, -2.0, 3.0).normalize(),
    ];
    let plain_error: f32 = probes
        .iter()
        .map(|&expected| {
            (plain
                .correct_working_for_test(offset + distortion * expected)
                .expect("plain working calibration is invalid")
                - expected)
                .norm()
        })
        .sum();
    let refined_error: f32 = probes
        .iter()
        .map(|&expected| {
            (gravity_refined
                .correct_working_for_test(offset + distortion * expected)
                .expect("gravity-refined working calibration is invalid")
                - expected)
                .norm()
        })
        .sum();

    assert!(
        refined_error < plain_error,
        "plain_error={plain_error} refined_error={refined_error}"
    );
}

#[test]
fn mag_calibrator_ignores_invalid_gravity() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut plain = MagCalibrator::<12>::new();
    let mut invalid = MagCalibrator::<12>::new();
    for i in 0..12 {
        let raw = offset + distortion * sample_direction(i, 12);
        let _ = plain.evaluate_correct(raw, None, i as u64);
        let gravity = match i % 3 {
            0 => Vector3::repeat(f32::NAN),
            1 => Vector3::repeat(f32::MAX),
            _ => Vector3::zeros(),
        };
        let _ = invalid.evaluate_correct(raw, Some(gravity), i as u64);
    }
    let training_updates = 2 * MIN_PUBLICATION_STREAK;
    for i in 12..12 + training_updates {
        let raw = offset + distortion * sample_direction(i % 12, 12);
        let _ = plain.evaluate_correct(raw, None, i as u64);
        let gravity = match i % 3 {
            0 => Vector3::repeat(f32::NAN),
            1 => Vector3::repeat(f32::MAX),
            _ => Vector3::zeros(),
        };
        let _ = invalid.evaluate_correct(raw, Some(gravity), i as u64);
    }

    let expected = Vector3::new(1.0, -2.0, 3.0).normalize();
    let raw = offset + distortion * expected;
    let plain = calibrated(plain.evaluate_correct(raw, None, 100));
    let invalid = calibrated(invalid.evaluate_correct(raw, None, 100));
    assert_vec_close(invalid, plain, 1.0e-6);
}

#[test]
fn mag_calibrator_uses_corrected_centered_covariance_for_coverage() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(2.4, 0.3, -0.2, 0.3, 0.7, 0.1, -0.2, 0.1, 1.3);
    let correction = distortion.try_inverse().unwrap();
    let mut calibrator = MagCalibrator::<63>::new();
    let mut directions = Vec::new();
    for i in 0..63 {
        let direction = sample_direction(i, 63);
        directions.push(direction);
        calibrator.evaluate_sample_vec(offset + distortion * direction, None, i as u64);
    }

    let mean = directions.iter().copied().sum::<Vector3<f32>>() / directions.len() as f32;
    let expected = directions.iter().fold(Matrix3::zeros(), |sum, direction| {
        let centered = direction - mean;
        sum + centered * centered.transpose()
    }) / directions.len() as f32;
    let actual = calibrator
        .corrected_covariance_for_test(correction)
        .expect("maintained covariance is unavailable");

    assert!(
        (actual - expected).norm() < 1.0e-5,
        "actual={actual:?} expected={expected:?}"
    );
}

#[test]
fn live_quality_does_not_read_the_sample_cache() {
    let offset = Vector3::new(11.0, -7.0, 5.0);
    let distortion = Matrix3::new(1.4, 0.2, -0.1, 0.2, 0.9, 0.15, -0.1, 0.15, 1.2);
    let mut calibrator = seeded_calibrator::<63>(offset, distortion);

    assert_eq!(calibrator.quality_cache_reads_for_test(), 0);
}

#[test]
fn live_quality_ramps_and_running_mean_square_match_the_specification() {
    let identity = Matrix3::identity();
    let condition_ten = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, 10.0));
    let condition_hundred = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, 100.0));
    let singular = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, 0.0));

    assert_eq!(
        MagCalibrator::<9>::quality_scores_for_test(identity, Some(0.0)),
        (1.0, 1.0)
    );
    let (coverage, fitness) =
        MagCalibrator::<9>::quality_scores_for_test(condition_ten, Some(0.05f32.powi(2)));
    assert!((coverage - 0.5).abs() < 1.0e-6, "coverage={coverage}");
    assert!((fitness - 0.5).abs() < 1.0e-6, "fitness={fitness}");
    assert_eq!(
        MagCalibrator::<9>::quality_scores_for_test(condition_hundred, Some(0.1f32.powi(2))),
        (0.0, 0.0)
    );
    assert_eq!(
        MagCalibrator::<9>::quality_scores_for_test(singular, Some(f32::NAN)),
        (0.0, 0.0)
    );
    assert_eq!(
        MagCalibrator::<9>::quality_scores_for_test(identity, Some(-1.0)),
        (1.0, 0.0)
    );
    assert_eq!(
        MagCalibrator::<9>::running_mean_square_for_test(None, 0.04, 0.25),
        Some(0.01)
    );
    let updated = MagCalibrator::<9>::running_mean_square_for_test(Some(0.04), 0.0, 0.25).unwrap();
    assert!((updated - 0.03).abs() < 1.0e-7, "updated={updated}");
    assert_eq!(
        MagCalibrator::<9>::running_mean_square_for_test(Some(f32::NAN), 0.0, 0.25),
        None
    );
    assert_eq!(
        MagCalibrator::<9>::running_mean_square_for_test(None, f32::INFINITY, 0.25),
        None
    );
}

#[test]
fn raw_moments_follow_all_cache_mutations_and_const_generic_edges() {
    let mut calibrator = MagCalibrator::<3>::new()
        .num_neighbors(2)
        .max_sample_lifespan_us(5);
    let samples = [Vector3::x(), Vector3::y(), Vector3::z()];
    for (timestamp_us, sample) in samples.into_iter().enumerate() {
        calibrator.evaluate_sample_vec(sample, None, timestamp_us as u64);
        calibrator.check_raw_moments().unwrap();
    }

    let full_moments = calibrator.raw_moments_for_test();
    calibrator.evaluate_sample_vec(Vector3::x(), None, 3);
    assert_eq!(calibrator.raw_moments_for_test(), full_moments);
    calibrator.check_raw_moments().unwrap();

    calibrator.evaluate_sample_vec(Vector3::repeat(10.0), None, 4);
    assert_ne!(calibrator.raw_moments_for_test(), full_moments);
    calibrator.check_raw_moments().unwrap();

    calibrator.evaluate_sample_vec(Vector3::repeat(f32::NAN), None, 7);
    assert_eq!(calibrator.raw_moments_for_test().0, 2);
    calibrator.check_raw_moments().unwrap();

    calibrator.evaluate_sample_vec(Vector3::repeat(f32::INFINITY), None, 10);
    assert_eq!(
        calibrator.raw_moments_for_test(),
        (0, Vector3::zeros(), Matrix3::zeros())
    );
    calibrator.check_raw_moments().unwrap();

    let mut empty = MagCalibrator::<0>::new();
    empty.evaluate_sample_vec(Vector3::x(), None, 0);
    empty.evaluate_sample_vec(Vector3::repeat(f32::NAN), None, 1);
    assert_eq!(
        empty.raw_moments_for_test(),
        (0, Vector3::zeros(), Matrix3::zeros())
    );
    empty.check_raw_moments().unwrap();

    let mut singleton = MagCalibrator::<1>::new().max_sample_lifespan_us(0);
    singleton.evaluate_sample_vec(Vector3::x(), None, 0);
    let singleton_moments = singleton.raw_moments_for_test();
    singleton.evaluate_sample_vec(Vector3::y(), None, 0);
    assert_eq!(singleton.raw_moments_for_test(), singleton_moments);
    singleton.evaluate_sample_vec(Vector3::repeat(f32::NAN), None, 1);
    assert_eq!(
        singleton.raw_moments_for_test(),
        (0, Vector3::zeros(), Matrix3::zeros())
    );
    singleton.check_raw_moments().unwrap();
}

#[test]
fn mag_calibrator_neighbor_cache_matches_naive_rescan() {
    // Deterministic xorshift64 PRNG.
    let mut prng_state = 0x9E37_79B9_7F4A_7C15u64;
    let mut next = move || {
        prng_state ^= prng_state << 13;
        prng_state ^= prng_state >> 7;
        prng_state ^= prng_state << 17;
        prng_state
    };

    // N - 1 = 11 exceeds the neighbor cache capacity of 8, so caches are
    // incomplete and inserts are dropped once the pad is exhausted; k = 10
    // additionally exercises the above-capacity direct-scan fallback. The
    // short lifespan keeps expiry compaction and buffer refills in the mix.
    for k in [2, 3, 10] {
        let mut calibrator = MagCalibrator::<12>::new()
            .num_neighbors(k)
            .max_sample_lifespan_us(25);
        let mut timestamp_us = 0;
        for _ in 0..4000 {
            timestamp_us += 1 + next() % 3;
            // Quantized directions with jitter: new samples frequently land
            // near buffered ones, provoking replacements.
            let theta = (next() % 8) as f32 * 0.785 + (next() % 100) as f32 / 500.0;
            let z = (next() % 5) as f32 / 2.5 - 1.0 + (next() % 100) as f32 / 500.0;
            let radius = (1.0 - z * z).max(0.0).sqrt();
            let direction = Vector3::new(radius * theta.cos(), radius * theta.sin(), z);
            let sample = Vector3::new(11.0, -7.0, 5.0) + 40.0 * direction;
            calibrator.evaluate_sample_vec(sample, None, timestamp_us);
            calibrator
                .check_neighbor_cache()
                .unwrap_or_else(|message| panic!("k={k} timestamp_us={timestamp_us}: {message}"));
            calibrator
                .check_raw_moments()
                .unwrap_or_else(|message| panic!("k={k} timestamp_us={timestamp_us}: {message}"));
        }
    }
}

fn seeded_calibrator<const N: usize>(
    offset: Vector3<f32>,
    distortion: Matrix3<f32>,
) -> MagCalibrator<N> {
    train_calibrator(MagCalibrator::new(), offset, distortion)
}

fn train_calibrator<const N: usize>(
    mut calibrator: MagCalibrator<N>,
    offset: Vector3<f32>,
    distortion: Matrix3<f32>,
) -> MagCalibrator<N> {
    for i in 0..N {
        let direction = sample_direction(i, N);
        let _ = calibrator.evaluate_correct(offset + distortion * direction, None, 0);
    }
    let mut result = None;
    let training_updates = (16 * N).max(2 * MIN_PUBLICATION_STREAK);
    for i in 0..training_updates {
        let direction = sample_direction(i % N, N);
        result = Some(calibrator.evaluate_correct(offset + distortion * direction, None, 0));
    }
    assert!(
        result.is_some_and(|result| result.is_ok_and(|result| result.calibrated().is_some())),
        "online calibration did not converge"
    );
    calibrator
}

fn sample_direction(i: usize, n: usize) -> Vector3<f32> {
    let theta = 0.37 + i as f32 * 1.21;
    let z = -0.8 + 1.6 * i as f32 / (n - 1) as f32;
    let radius = (1.0 - z * z).sqrt();
    Vector3::new(radius * theta.cos(), radius * theta.sin(), z)
}

fn assert_vec_close(actual: Vector3<f32>, expected: Vector3<f32>, tolerance: f32) {
    let diff = (actual - expected).norm();
    assert!(
        diff < tolerance,
        "actual={} expected={} diff={}",
        actual.transpose(),
        expected.transpose(),
        diff
    );
}

fn calibrated(result: Result<MagCalibrationResult, BadMagCause>) -> Vector3<f32> {
    result
        .expect("magnetometer evaluation failed")
        .calibrated()
        .expect("calibration is still pending")
}
