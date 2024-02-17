use crate::{any_glasses, ARGlasses, Error, GlassesEvent};
use nalgebra::{UnitQuaternion, Vector3};
use std::ops::Deref;

type Result<T> = std::result::Result<T, Error>;

mod c_f {
    /*
    complementary filter with very simple update algorithm:

    (
        assuming:
        (S, S-) = current & previous estimated state,
        d~S1 = rate sensor reading (e.g. gyro), high frequency, high drift, dead reckoning,
        ~S2 = state sensor reading (e.g. grav/acc, mag), low frequency, high noise, low drift
        d_t1 = time elapse since last rate sensor sampling
    )

    S = S- + ratio * d~S1 * d_t1 + (1-ratio) * (~S2 - S-)
      = ratio * (S- + d~S1 * d_t1) + (1-ratio) * ~S2

    this implies:

    - the algorithm natively support state sensor(s) with different sampling frequency, incomplete reading,
    or unreliable reading, as ~S2 variable is merely an optional correction
    - the interpolation between ~S2 and the first term doesn't need to be linear or additive,
    e.g. 3D angular interpolation is multiplicative
    - the ratio can be adjusted based on quality & frequency of state sensor(s)

    most glasses have acc & grav/acc readings in 1 bundle, but I prefer not using this assumption and still update them independently

     */
    pub const BASE_GRAV_RATIO: f32 = 0.6;
    pub const BASE_MAG_RATIO: f32 = 0.5;
}

/*
high level interface of glasses & state estimation, with the following built-in fusion pipeline:

(the first version should only use complementary filter for simplicity and sanity test)

- roll/pitch <= acc + gyro (complementary filter)
  - assuming that acc vector always pointed up, spacecraft moving in that direction can create 1G artificial gravity
    - TODO: this obviously assumes no negative/high G manoeuvre, at which point up d_acc has to be used to get the right up direction
  - TODO: use ESKF (error-state/multiplicatory KF, https://arxiv.org/abs/1711.02508)
- gyro-yaw <= gyro (integrate over time)
- mag-yaw <= mag + roll/pitch (arctan)
  - TODO: mag calibration?
     (continuous ellipsoid fitting, assuming homogeneous E-M environment & hardpoint-mounted E-M interference)
- yaw <= mag-yaw + gyro-gyro (complementary filter)
  - TODO: use EKF
*/
pub struct Connection {
    pub glasses: Box<dyn ARGlasses>,
    // estimation
    pub attitude: UnitQuaternion<f32>,
    // just old readings
    // prevAcc: (Vector3<f32>, u64),
    prev_gyro: (Vector3<f32>, u64),
    // prevMag: (Vector3<f32>, u64),
}

pub fn default() -> Result<Connection> {
    let mut glasses = any_glasses()?;
    loop {
        // wait for the first non-zero gyro reading
        let next_event = glasses.next_event();
        match next_event {
            GlassesEvent::AccGyro {
                accelerometer: _,
                gyroscope,
                timestamp,
            } => {
                if gyroscope != Vector3::zeros() {
                    let conn = Connection {
                        glasses,
                        attitude: UnitQuaternion::identity(),
                        prev_gyro: (gyroscope, timestamp),
                    };
                    return Ok(conn);
                }
            }
            _ => {}
        }
    }
}

impl Connection {
    const UP: Vector3<f32> = Vector3::new(0.0, 1.0, 0.0);
    const NORTH: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);

    fn disconnect(self) -> () {
        self.glasses.deref();
    }

    fn update_gyro(&mut self, gyro: Vector3<f32>, t: u64) -> () {
        let d_t1 = ((t - self.prev_gyro.1) as f32) / 1000.0;
        let d_S1_t1 = gyro * d_t1;

        let integrated =
            self.attitude * UnitQuaternion::from_euler_angles(d_S1_t1.y, d_S1_t1.x, d_S1_t1.z);

        self.attitude = integrated;
        self.prev_gyro = (gyro, t);
    }

    fn update_acc(&mut self, acc: Vector3<f32>, _t: u64) -> () {
        let uncorrected = self.attitude * Connection::UP;

        let delta = UnitQuaternion::rotation_between(&uncorrected, &acc);

        match delta {
            Option::Some(d) => {
                let corrected = self.attitude
                    * UnitQuaternion::nlerp(
                        &UnitQuaternion::identity(),
                        &d,
                        1.0 - c_f::BASE_GRAV_RATIO,
                    );

                self.attitude = corrected;
            }
            Option::None => {} // no update
        }
    }

    pub fn update(&mut self) -> () {
        let event = self.glasses.next_event();
        match event {
            GlassesEvent::AccGyro {
                accelerometer,
                gyroscope,
                timestamp,
            } => {
                self.update_gyro(gyroscope, timestamp);
                self.update_acc(accelerometer, timestamp);
            }
            _ => {
                // TODO: add magnetometer event etc
            }
        }
    }
}
