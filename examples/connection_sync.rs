// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::connection::{Connection};
use nalgebra::Vector3;
use std::slice;
use ar_drivers::ffi::{GetEuler, StartConnection, StopConnection};

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
            // let quaternion = {
            //     let ptr = GetQuaternion();
            //     // Array::from_raw_parts(ptr, 4);
            //     unsafe { slice::from_raw_parts(ptr, 4) }
            // };
            let frd = {
                let ptr = GetEuler();
                unsafe {
                    let slice = slice::from_raw_parts(ptr, 3);
                    Vector3::new(slice[0], slice[1], slice[2])
                }
            };
            println!("euler:\t{:10.7}(i={})", frd.transpose(), _i);
            // println!("quaternion:\t{:?}\teuler:\t{:?}", quaternion, euler);
        }

        let code = StopConnection();
        println!("\nstopping, code {}", code);
    }
}
