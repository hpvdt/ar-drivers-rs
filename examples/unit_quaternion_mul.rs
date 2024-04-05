use clap::Parser;

use nalgebra::{UnitQuaternion, Vector3};

fn main() {
    use rand::Rng;

    let mut rng = rand::thread_rng();

    let mut qq = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);

    loop {
        let u1 = rng.gen_range(-1.58f32..1.58);
        let u2 = rng.gen_range(-1.58f32..1.58);
        let u3 = rng.gen_range(-1.58f32..1.58);

        let random = UnitQuaternion::from_euler_angles(u1, u2, u3);

        qq = random * qq;

        let norm = qq.quaternion().norm();

        let error = { norm - 1.0 }.abs();

        println!("error={}", error);

        assert!(error < 0.0001);
    }
}
