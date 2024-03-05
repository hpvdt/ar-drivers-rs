// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use std::time::{Duration, Instant};

use ar_drivers::{any_fusion, any_glasses, GlassesEvent};
use nalgebra::Vector3;

fn main() {
    let mut conn = any_fusion().unwrap();

    println!("Got glasses, serial={}", conn.glasses.serial().unwrap());

    loop {
        let quaternion = conn.attitude_quaternion();
        let euler = conn.attitude_euler();

        println!("attitude:\n{}\neuler:\n{}", quaternion, euler);
    }
}
