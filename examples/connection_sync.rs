// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::connection::Connection;
use ar_drivers::ffi::{GetEuler, StartConnection, StopConnection};
use ar_drivers::Fusion;
use nalgebra::Vector3;

fn main() {
    // stress test to ensure connection after disconnection works
    for _a in 0..100 {
        let code = StartConnection();
        println!("starting, code {}", code);

        {
            let _serial = Connection::read_fusion(&mut |fusion| {
                let serial = fusion.glasses().serial().unwrap();
                println!("Got glasses, serial={}", serial);
            });
        }

        println!("");

        for _i in 0..500 {
            let euler = GetEuler();
            let frd = Vector3::new(euler.x, euler.y, euler.z);

            println!(
                "euler:\t{:10.7}(i={}, code={})",
                frd.transpose(),
                _i,
                euler.success
            );
        }

        let code = StopConnection();
        println!("\nstopping, code {}", code);
    }
}
