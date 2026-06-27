/*
high level interface of glasses & state estimation, with the following built-in fusion pipeline:

(the first version should only use complementary filter for simplicity and sanity test)

- roll/pitch <= acc + gyro (complementary filter)
  - assuming that acc vector always pointed up, spacecraft moving in that direction can create 1G artificial gravity
    - TODO: this obviously assumes no steadily accelerating frame, at which point up d_acc has to be used for correction
  - TODO: use ESKF (error-state/multiplicatory KF, https://arxiv.org/abs/1711.02508)
- gyro-yaw <= gyro (integrate over time)
- mag-yaw <= mag + roll/pitch (arctan)
  - TODO: mag calibration?
     (continuous ellipsoid fitting, assuming homogeneous E-M environment & hardpoint-mounted E-M interference)
- yaw <= mag-yaw + gyro-gyro (complementary filter)
  - TODO: use EKF

CAUTION: unlike [[GlassesEvent]], all states & outputs should use FRD reference frame
 (forward, right, down, corresponding to roll, pitch, yaw in Euler angles-represented rotation)

FRD is the standard frame for aerospace, and is also the default frame for NALgebra
*/
use std::f32::consts::PI;
use std::fmt;

use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};

use self::naive_cf::NaiveCF;
use crate::fusion::mag_calibration::MagCalibrator;
use crate::{any_glasses_or_dummy, ARGlasses, Result};

mod mag_calibration;
#[cfg(test)]
mod mag_calibration_test;
mod naive_cf;
#[cfg(test)]
mod naive_cf_test;

/// Non-overridable fusion inconsistency computation.
pub trait FusionInconsistency {
    /// use FRD frame as error in Quaternion is multiplicative & is over-defined
    fn inconsistency(&self) -> f32;
}

impl<T: Fusion + ?Sized> FusionInconsistency for T {
    fn inconsistency(&self) -> f32 {
        self.corrections().totalAvg()
    }
}

pub trait Fusion: Send + FusionInconsistency {
    fn glasses(&mut self) -> &mut Box<dyn ARGlasses>;
    // TODO: only declared mutable as many API of ARGlasses are also mutable

    /// primary estimation output
    /// can be used to convert to Euler angles of different conventions
    fn attitude_quaternion(&self) -> UnitQuaternion<f32>;

    /// Per-sensor correction magnitudes tracked by the fusion algorithm.
    fn corrections(&self) -> NineAxis<Correction>;

    fn update(&mut self) -> ();
}

impl dyn Fusion {
    pub fn any_cf() -> Result<Box<dyn Fusion>> {
        // let glasses = any_glasses()?;
        let glasses = any_glasses_or_dummy()?;
        Ok(Box::new(NaiveCF::new(glasses)?))
    }
}

/// Last and averaged correction magnitudes for a sensor.
#[derive(Clone, Copy, Debug)]
pub struct Correction {
    /// Most recent correction magnitude in radians.
    pub prev: f32, // previous
    /// Exponential moving average of correction magnitude in radians.
    pub avg: f32, // average
}

impl Correction {
    /// Exponential averaging decay rate.
    pub const AVG_DECAY: f32 = 0.90;

    fn new() -> Self {
        Self {
            prev: 0.0,
            avg: 0.0,
        }
    }

    fn record(&mut self, correction: f32) -> () {
        self.prev = correction;
        self.avg = self.avg * Self::AVG_DECAY + correction * (1.0 - Self::AVG_DECAY);
    }
}

impl Default for Correction {
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Display for Correction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "prev={:10.7}, avg={:10.7}", self.prev, self.avg)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct NineAxis<T> {
    /// Accelerometer correction.
    pub acc: T,
    /// Gyroscope integration increment.
    pub gyro: T,
    /// Magnetometer correction.
    pub mag: T,
}

impl NineAxis<Correction> {
    fn totalAvg(&self) -> f32 {
        self.acc.avg + self.gyro.avg + self.mag.avg
    }
}

impl fmt::Display for NineAxis<Correction> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "acc: {}, gyro: {}, mag: {}",
            self.acc, self.gyro, self.mag
        )
    }
}

pub struct FusionState {
    pub glasses: Box<dyn ARGlasses>,

    // following data will be updated in memory directly,
    pub attitude: UnitQuaternion<f32>,

    /// Per-sensor correction magnitudes.
    pub corrections: NineAxis<Correction>,

    // mag calibration state, will be used by all Fusion impls
    pub mag: MagCalibrator<63>,
}

impl FusionState {
    const MIN_MAG_NORM: f32 = 0.4;

    /// Creates a shared fusion state with identity attitude and empty calibration state.
    pub fn new(glasses: Box<dyn ARGlasses>) -> Self {
        Self {
            glasses,
            attitude: UnitQuaternion::identity(),
            corrections: NineAxis::default(),
            mag: MagCalibrator::new(),
        }
    }

    fn getCalibratedMag(&mut self, raw_mag: Vector3<f32>) -> Option<Vector3<f32>> {
        if raw_mag.norm() < Self::MIN_MAG_NORM {
            return None;
        }

        self.mag.evaluate_sample_vec(raw_mag);
        let calibration = self.mag.perform_calibration();
        let mag: Vector3<f32> = match calibration {
            Some((offset, scale)) => {
                let offset: Vector3<f32> = Vector3::from(offset);
                let scale: Vector3<f32> = Vector3::from(scale);
                (raw_mag - offset).component_div(&scale)
            }
            None => raw_mag,
        };

        if mag.norm() < Self::MIN_MAG_NORM {
            None
        } else {
            Some(mag.normalize())
        }
    }
}

pub struct AhrsCorrection {
    fusion: Box<dyn Fusion>,
    neutral_bias: (UnitQuaternion<f32>, UnitQuaternion<f32>), // multiplicative
    euler_bias: Vector3<f32>,
    euler_order: Vector3<i16>, // each element represents an index + sign of an Euler angle axis
    quaternion_bias: (UnitQuaternion<f32>, UnitQuaternion<f32>),
    quaternion_order: Vector4<i16>,
}

impl AhrsCorrection {
    // default reference system used by most robotics applications
    //
    // Euler-Angle: forward-right-down, right hand axes, right hand rotation
    //
    // Quaternion: right hand, ijkw
    pub fn frd(fusion: Box<dyn Fusion>) -> AhrsCorrection {
        AhrsCorrection {
            // defaults to RUF reference frame of Unity game engine
            fusion,
            neutral_bias: (AhrsCorrection::q_id(), AhrsCorrection::q_id()),
            euler_bias: Vector3::new(0.0, 0.0, 0.0),
            euler_order: Vector3::new(1, 2, 3),
            quaternion_bias: (AhrsCorrection::q_id(), AhrsCorrection::q_id()),
            quaternion_order: Vector4::new(1, 2, 3, 4),
        }
    }

    // reference system used by AirAPI_Windows
    // neutral heading is 90 degree pitch down
    //
    // Euler-Angle: forward-right-up, left hand axes, right hand rotation
    // CAUTION: this is a heterochiral system (using different hands for axes and rotation)
    //   in practice it should be avoided due to being highly corruptive
    //   but here it is used for backward compatibility
    //
    // Quaternion: left hand, ijkw ?
    pub fn left_fru_down(fusion: Box<dyn Fusion>) -> AhrsCorrection {
        AhrsCorrection {
            // defaults to RUF reference frame of Unity game engine
            fusion,
            // neutral_bias: UnitQuaternion::from_euler_angles(0.0, PI * 0.5, 0.0),
            neutral_bias: (AhrsCorrection::q_id(), AhrsCorrection::q_id()),
            euler_bias: Vector3::new(0.0, PI * 0.5, 0.0),
            euler_order: Vector3::new(1, 2, -3),
            quaternion_bias: (
                // UnitQuaternion::from_euler_angles(0.0, PI * 0.5, 0.0),
                AhrsCorrection::q_id(),
                UnitQuaternion::from_euler_angles(0.0, PI * 0.5, 0.0),
                // AHRS::q_id(),
            ),
            quaternion_order: Vector4::new(2, 1, -3, 4),
        }
    }

    fn q_id() -> UnitQuaternion<f32> {
        UnitQuaternion::identity()
    }

    fn attitude_quaternion_frd(&self) -> UnitQuaternion<f32> {
        let original = self.fusion.attitude_quaternion();
        let corrected = self.neutral_bias.0 * original * self.neutral_bias.1;
        corrected
    }

    pub fn attitude_euler_rad(&self) -> Vector3<f32> {
        let (roll, pitch, yaw) = self.attitude_quaternion_frd().euler_angles();
        let frd = Vector3::new(roll, pitch, yaw);
        let biased = frd + self.euler_bias;
        let ordered = self.euler_order.map(|v| {
            let index = (v.abs() - 1) as usize;
            let vv = biased.get(index).unwrap();
            let vv_with_sign = vv * (v.signum() as f32);
            vv_with_sign
        });
        ordered
    }

    pub fn attitude_euler_deg(&self) -> Vector3<f32> {
        self.attitude_euler_rad().map(|x| x.to_degrees())
    }

    /// Returns the non-overridable fusion inconsistency computation.
    pub fn inconsistency(&self) -> f32 {
        <Self as FusionInconsistency>::inconsistency(self)
    }
}

impl Fusion for AhrsCorrection {
    fn glasses(&mut self) -> &mut Box<dyn ARGlasses> {
        self.fusion.glasses()
    }

    fn corrections(&self) -> NineAxis<Correction> {
        self.fusion.corrections()
    }

    fn update(&mut self) -> () {
        self.fusion.update()
    }

    fn attitude_quaternion(&self) -> UnitQuaternion<f32> {
        let q_raw = self.attitude_quaternion_frd();

        let q_biased = self.quaternion_bias.0 * q_raw * self.quaternion_bias.1;

        // let q_biased = q_raw * self.quaternion_bias;
        let biased = q_biased.coords;

        let ordered = self.quaternion_order.map(|v| {
            let index = (v.abs() - 1) as usize;
            let vv = biased.get(index).unwrap();
            let vv_with_sign = vv * (v.signum() as f32);
            vv_with_sign
        });

        let q_ordered = UnitQuaternion::from_quaternion(Quaternion::new(
            ordered[3], ordered[0], ordered[1], ordered[2],
        ));

        q_ordered
    }
}
