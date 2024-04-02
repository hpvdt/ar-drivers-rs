use crate::{ARGlasses, Error, Fusion, GlassesEvent};
use nalgebra::{UnitQuaternion, Vector3};

type Result<T> = std::result::Result<T, Error>;

/*
complementary filter with very simple update algorithm:

(
    assuming:
    (S, S-) = current & previous estimated state
    d~S1 = rate sensor reading (e.g. gyro), high frequency, high drift, dead reckoning
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
pub struct NaiveCF {
    pub glasses: Box<dyn ARGlasses>,
    // estimation
    pub attitude: UnitQuaternion<f32>,
    // just old readings
    // prevAcc: (Vector3<f32>, u64),
    pub prev_gyro: (Vector3<f32>, u64), // FRD
    // prevMag: (Vector3<f32>, u64),
    pub inconsistency: f32, // roll, pitch. yaw
}

impl NaiveCF {
    pub fn new(glasses: Box<dyn ARGlasses>) -> Result<Self> {
        // let attitude = ;
        // let prev_gyro = ;
        let mut fusion = NaiveCF {
            glasses,
            attitude: UnitQuaternion::identity(),
            prev_gyro: (Vector3::zeros(), 0),
            inconsistency: 0.0,
        };

        loop {
            // wait for the first non-zero gyro reading
            let next_event = fusion.next_event();
            match next_event {
                GlassesEvent::AccGyro {
                    accelerometer: _,
                    gyroscope,
                    timestamp,
                } => {
                    // if gyroscope != Vector3::zeros() {
                    fusion.prev_gyro = (gyroscope, timestamp);
                    return Ok(fusion);
                    // }
                }
                _ => {}
            }
        }
    }

    /// read until next valid event. Blocks.
    fn next_event(&mut self) -> GlassesEvent {
        loop {
            match self.glasses.read_event() {
                Ok(event) => return event,
                Err(e) => {
                    println!("Error reading event: {}", e);
                }
            }
        }
    }

    fn rub_to_frd(v: &Vector3<f32>) -> Vector3<f32> {
        let result = Vector3::new(-v.z, v.x, -v.y);
        result
    }

    // const BASE_GRAV_RATIO: f32 = 0.005;
    // const BASE_GRAV_RATIO: f32 = 0.0; // no grav
    const BASE_GRAV_RATIO: f32 = 1.0; // absolute correction, no gyro

    // const BASE_MAG_RATIO: f32 = 0.5;

    const GYRO_SPEED_IN_TIMESTAMP_FACTOR: f32 = 1000.0 * 1000.0; // microseconds

    const INCONSISTENCY_DECAY: f32 = 0.90;

    const UP_FRD: Vector3<f32> = Vector3::new(0.0, 0.0, -9.81);
    // const NORTH_FRD: Vector3<f32> = Vector3::new(0.0, 0.0, -1.0);

    // CAUTION: right-multiplication means rotation, unconventionally

    fn update_gyro_rub(&mut self, gyro_rub: &Vector3<f32>, t: u64) -> () {
        let gyro = Self::rub_to_frd(gyro_rub);

        let d_t1 = t - self.prev_gyro.1;
        let d_t1_f = d_t1 as f32 / Self::GYRO_SPEED_IN_TIMESTAMP_FACTOR;
        let d_s1_t1 = d_t1_f * gyro;

        let integrated =
            self.attitude * UnitQuaternion::from_euler_angles(d_s1_t1.x, d_s1_t1.y, d_s1_t1.z);

        self.attitude = integrated;
        self.prev_gyro = (gyro, t);
    }

    fn dummy(&mut self) -> () {
        let mut a = 0;

        let delta_opt = || -> (Option<f32>) {
            let a = self.attitude.inverse();
            Some(a.i)
        };
        match delta_opt() {
            Some(v) => {
                self.attitude = UnitQuaternion::identity();
            }
            None => {}
        }
    }

    fn update_acc(&mut self, acc_rub: &Vector3<f32>, _t: u64) -> () {
        let acc = Self::rub_to_frd(acc_rub);

        if acc.norm() < 1.0 {
            return (); // almost in free fall, or acc disabled, do not correct
        }

        let delta_opt = || -> Option<UnitQuaternion<f32>> {
            let uncorrected = self.attitude.inverse().transform_vector(&Self::UP_FRD);
            let delta_opt = UnitQuaternion::rotation_between(&acc, &uncorrected);

            delta_opt
        };

        let dd = delta_opt();
        match dd {
            Some(delta) => {
                self.inconsistency = self.inconsistency * Self::INCONSISTENCY_DECAY + delta.angle();

                match UnitQuaternion::try_slerp(
                    &UnitQuaternion::identity(),
                    &delta,
                    Self::BASE_GRAV_RATIO,
                    0.01,
                ) {
                    Some(correction) => {
                        let new_attitude = self.attitude * correction;
                        self.attitude = new_attitude;

                        {
                            // TODO: verification code, clean up
                            let residual = delta_opt().unwrap().angle().abs();
                            if (residual >= 0.01) {
                                println!("!!!!!!!!!! residual: {} !!!!!!!!!!", residual);
                            }
                        }
                    }
                    None => {
                        // TODO: opposite direction, don't know how to correct
                    }
                };
            }
            None => {
                // no error no update
            }
        }
    }
}

// unsafe impl Sync for NaiveCF {}

impl Fusion for NaiveCF {
    fn glasses(&mut self) -> &mut Box<dyn ARGlasses> {
        &mut self.glasses
    }

    fn attitude_quaternion(&self) -> UnitQuaternion<f32> {
        self.attitude
    }

    fn inconsistency_frd(&self) -> f32 {
        self.inconsistency
    }

    fn update(&mut self) -> () {
        let event = self.next_event();
        match event {
            GlassesEvent::AccGyro {
                accelerometer,
                gyroscope,
                timestamp,
            } => {
                // self.update_gyro_rub(&gyroscope, timestamp);
                self.update_acc(&accelerometer, timestamp);
            }
            _ => {
                // TODO: add magnetometer event etc
            }
        }
    }
}
