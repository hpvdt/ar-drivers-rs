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
    pub prev_gyro: (Vector3<f32>, u64), // roll, pitch, yaw
    // prevMag: (Vector3<f32>, u64),
    pub inconsistency: Vector3<f32>, // roll, pitch. yaw
}

impl NaiveCF {
    const BASE_GRAV_RATIO: f32 = 0.6;
    // const BASE_MAG_RATIO: f32 = 0.5;

    const GYRO_SPEED_IN_TIMESTAMP_FACTOR: f32 = 1000.0 * 1000.0; // microseconds

    const INCONSISTENCY_DECAY: f32 = 0.99;

    const UP_RUB: Vector3<f32> = Vector3::new(0.0, 9.81, 0.0);
    const NORTH_RUB: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);

    pub fn new(glasses: Box<dyn ARGlasses>) -> Result<Self> {
        // let attitude = ;
        // let prev_gyro = ;
        let mut fusion = NaiveCF {
            glasses,
            attitude: UnitQuaternion::identity(),
            prev_gyro: (Vector3::zeros(), 0),
            inconsistency: Vector3::zeros(),
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

    fn update_gyro_rub(&mut self, gyro: Vector3<f32>, t: u64) -> () {
        let d_t1 = t - self.prev_gyro.1;
        let d_t1_f = d_t1 as f32 / Self::GYRO_SPEED_IN_TIMESTAMP_FACTOR;
        let d_s1_t1 = gyro * d_t1_f;

        let d_s1_t1_frd = Vector3::new(-d_s1_t1.z, d_s1_t1.x, -d_s1_t1.y);

        let integrated = self.attitude
            * UnitQuaternion::from_euler_angles(d_s1_t1_frd.x, d_s1_t1_frd.y, d_s1_t1_frd.z);

        self.attitude = integrated;
        self.prev_gyro = (gyro, t);
    }

    fn update_acc(&mut self, acc: Vector3<f32>, _t: u64) -> () {
        let uncorrected = self.attitude.conjugate() * Self::UP_RUB;

        let delta_opt = UnitQuaternion::rotation_between(&uncorrected, &acc);

        match delta_opt {
            Some(delta) => {
                let (roll, pitch, yaw) = delta.euler_angles();
                self.inconsistency = self.inconsistency * Self::INCONSISTENCY_DECAY
                    + Vector3::new(roll, pitch, yaw) * (1.0 - Self::INCONSISTENCY_DECAY);

                let corrected = self.attitude
                    * UnitQuaternion::nlerp(
                        &UnitQuaternion::identity(),
                        &delta,
                        1.0 - Self::BASE_GRAV_RATIO,
                    );

                self.attitude = corrected;
            }
            None => {
                // no error no update
            }
        }
    }
}

impl Fusion for NaiveCF {
    fn glasses(&mut self) -> &mut Box<dyn ARGlasses> {
        &mut self.glasses
    }

    fn attitude_quaternion(&self) -> UnitQuaternion<f32> {
        self.attitude
    }

    fn inconsistency_frd(&self) -> Vector3<f32> {
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
                self.update_gyro_rub(gyroscope, timestamp);
                // self.update_acc(accelerometer, timestamp);
            }
            _ => {
                // TODO: add magnetometer event etc
            }
        }
    }
}
