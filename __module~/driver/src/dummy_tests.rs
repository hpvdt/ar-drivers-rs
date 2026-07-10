use super::*;
use crate::fusion::rub_to_frd;

fn quiet_config() -> DummyConfig {
    DummyConfig {
        max_body_rate_rpm: 0.0,
        linear_jerk_std_dev: 0.0,
        gyro_noise_std_dev: 0.0,
        acc_noise_std_dev: 0.0,
        mag_noise_std_dev: 0.0,
        magnetic_dip_rad: 0.0,
        hard_iron_base: ZERO,
        hard_iron_drift: ZERO,
        soft_iron_min_eigenvalue: 1.0,
        soft_iron_max_eigenvalue: 1.0,
        ..DummyConfig::default()
    }
}

#[test]
fn starts_level_with_felt_gravity_up_frd() {
    let mut dummy = Dummy::with_config(quiet_config());

    match dummy.read_event().unwrap() {
        GlassesEvent::AccGyro {
            accelerometer,
            timestamp,
            ..
        } => {
            assert_eq!(timestamp, 0);
            let acc_frd = rub_to_frd(&accelerometer);
            let expected = Vector3::new(0.0, 0.0, -9.81);

            assert!(
                (acc_frd - expected).norm() < 1.0e-5,
                "acc_frd={:?}",
                acc_frd
            );
        }
        event => panic!("expected AccGyro, got {:?}", event),
    }
}

#[test]
fn starts_with_magnetic_north_forward_frd() {
    let mut dummy = Dummy::with_config(quiet_config());
    let _ = dummy.read_event().unwrap();

    match dummy.read_event().unwrap() {
        GlassesEvent::Magnetometer {
            magnetometer,
            timestamp,
        } => {
            assert_eq!(timestamp, 10_000);
            let mag_frd = rub_to_frd(&magnetometer);
            let expected = Vector3::new(50.0, 0.0, 0.0);

            assert!(
                (mag_frd - expected).norm() < 1.0e-5,
                "mag_frd={:?}",
                mag_frd
            );
        }
        event => panic!("expected Magnetometer, got {:?}", event),
    }
}

#[test]
fn default_hard_iron_bias_is_under_50_microtesla() {
    let hard_iron = Dummy::new().snapshot().hard_iron;

    assert!(
        hard_iron.norm() < 50.0,
        "hard_iron={:?}, norm={}",
        hard_iron,
        hard_iron.norm()
    );
}

#[test]
fn soft_iron_is_fixed_positive_definite_and_bounded() {
    let mut dummy = Dummy::new();
    let config = dummy.config.clone();
    let lower_eigenvalue = config.soft_iron_min_eigenvalue;
    let upper_eigenvalue = config.soft_iron_max_eigenvalue;
    let initial_soft_iron = dummy.snapshot().soft_iron;

    for sample in 0..=360 {
        dummy.timestamp_us = config.hard_iron_drift_period_us * sample / 360;
        let soft_iron = dummy.snapshot().soft_iron;
        let asymmetry = soft_iron - soft_iron.transpose();

        assert_eq!(soft_iron, initial_soft_iron);
        assert!(asymmetry.norm() <= 1.0e-5, "soft_iron={:?}", soft_iron);
        assert!(soft_iron.cholesky().is_some(), "soft_iron={:?}", soft_iron);

        for eigenvalue in soft_iron.symmetric_eigen().eigenvalues.iter() {
            assert!(
                *eigenvalue >= lower_eigenvalue - 1.0e-4
                    && *eigenvalue <= upper_eigenvalue + 1.0e-4,
                "eigenvalue={}, soft_iron={:?}",
                eigenvalue,
                soft_iron
            );
        }
    }
}

#[test]
fn default_gyro_body_rates_are_nonnegative() {
    let angular_rate = Dummy::new().snapshot().angular_rate_rub;
    let max_rad_per_sec = 6.0 * 2.0 * PI / SECONDS_PER_MINUTE;

    for component in angular_rate.iter() {
        assert!(
            *component >= 0.0 && *component <= max_rad_per_sec,
            "angular_rate={:?}",
            angular_rate
        );
    }
}
