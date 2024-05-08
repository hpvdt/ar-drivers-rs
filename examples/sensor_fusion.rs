// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::Fusion;
use ar_drivers::AHRS;

fn main() {
    let mut fusion = <dyn Fusion>::any_cf().unwrap(); // Declare conn as mutable
    let serial = fusion.glasses().serial().unwrap();

    println!("Got glasses, serial={}", serial);
    println!("");

    // let mut ahrs = AHRS::frd(fusion);
    let mut ahrs = AHRS::left_fru_down(fusion);

    loop {
        ahrs.update();
        let quaternion = ahrs.attitude_quaternion();
        let frd = ahrs.attitude_euler_deg();
        let inconsistency = ahrs.inconsistency();

        println!(
            "quaternion:\t{:10.7}\t:\t{:10.7}\t{:10.7}\t{:10.7}\t{:10.7}",
            quaternion, quaternion.i, quaternion.j, quaternion.k, quaternion.w
        );
        println!("euler:\t{:10.7}", frd.transpose());

        println!("inconsistency:\t{:10.7}", inconsistency);
    }
}
