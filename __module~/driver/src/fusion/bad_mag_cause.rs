use nalgebra::Vector3;

/// Reason a magnetometer vector could not produce a calibrated FRD reading.
#[derive(Clone, Copy, derive_more::Debug, derive_more::From, PartialEq)]
pub enum BadMagCause {
    /// The magnetometer calibration is not usable.
    BadCalibration(BadCalibration),
    /// The magnetometer reading is not usable.
    BadReading(BadReading),
}

/// Reason a magnetometer calibration is not usable.
#[derive(Clone, Copy, derive_more::Debug, PartialEq)]
pub enum BadCalibration {
    /// Not enough accepted samples have been collected for the calibration model.
    InsufficientSamples {
        /// Number of accepted samples currently available.
        samples: usize,
        /// Minimum number of samples required by the calibration model.
        required: usize,
    },
    /// The SVD-based calibration solve failed after the sample checks passed.
    Unsolveable {
        /// Original rejection message.
        message: &'static str,
    },
    /// The accepted samples produce a numerically unstable calibration solve.
    DegenerateSoftIronMatrix {
        /// Estimated condition number of the calibration design matrix.
        #[debug("{:+10.4}", condition)]
        condition: f32,
        /// Maximum accepted condition number.
        #[debug("{:+10.4}", max_condition)]
        max_condition: f32,
    },
    /// The calibration solve produced a non-finite offset or scale.
    DegenerateScale {
        /// Calibration offset.
        #[debug(
            "[x={:+10.4}, y={:+10.4}, z={:+10.4}]",
            offset.x,
            offset.y,
            offset.z
        )]
        offset: Vector3<f32>,
        /// Calibration scale.
        #[debug(
            "[x={:+10.4}, y={:+10.4}, z={:+10.4}]",
            scale.x,
            scale.y,
            scale.z
        )]
        scale: Vector3<f32>,
    },
    /// The accepted calibration cannot be safely applied.
    NumericallyUnstable {
        /// Calibration offset.
        #[debug(
            "[x={:+10.4}, y={:+10.4}, z={:+10.4}]",
            offset.x,
            offset.y,
            offset.z
        )]
        offset: Vector3<f32>,
        /// Calibration scale.
        #[debug(
            "[x={:+10.4}, y={:+10.4}, z={:+10.4}]",
            scale.x,
            scale.y,
            scale.z
        )]
        scale: Vector3<f32>,
    },
}

/// Reason a magnetometer reading is not usable.
#[derive(Clone, Copy, derive_more::Debug, PartialEq)]
pub enum BadReading {
    /// The raw magnetometer vector is too small to be useful. TODO: this is actually not a problem as hard-iron zero can be very far
    // WeakRawReading {
    //     /// Actual raw vector norm.
    //     norm: f32,
    //     /// Minimum accepted vector norm.
    //     min_norm: f32,
    // },
    /// The calibrated vector is too small to be useful.
    WeakCalibratedReading {
        /// Actual calibrated vector norm.
        #[debug("{:+10.4}", norm)]
        norm: f32,
        /// Minimum accepted vector norm.
        #[debug("{:+10.4}", min_norm)]
        min_norm: f32,
    },
}
