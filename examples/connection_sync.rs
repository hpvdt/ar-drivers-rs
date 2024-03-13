// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::{Connection, GetEuler, GetQuaternion, StartConnection, StopConnection};
use std::slice;

fn main() {
    let code = StartConnection();
    println!("starting, code {}", code);

    {
        let mut lock = Connection::instance().fusion.lock().unwrap();

        let mut fusion = lock.as_mut().unwrap();

        let serial = fusion.glasses().serial().unwrap();
        println!("Got glasses, serial={}", serial);
    }

    for i in (0..50000) {
        let quaternion = {
            let ptr = GetQuaternion();
            // Array::from_raw_parts(ptr, 4);
            unsafe { slice::from_raw_parts(ptr, 4) }
        };
        let euler = {
            let ptr = GetEuler();
            unsafe { slice::from_raw_parts(ptr, 3) }
        };
        println!("quaternion:\t{:?}\teuler:\t{:?}", quaternion, euler);
    }

    let code = StopConnection();
    println!("stopping, code {}", code);
}
