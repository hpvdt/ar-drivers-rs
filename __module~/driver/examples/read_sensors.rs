// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::any_glasses_or_dummy;
use ar_drivers::fusion::FusionState;

fn main() {
    let mut glasses = any_glasses_or_dummy().unwrap();
    let serial = glasses.serial().unwrap();
    println!("Got glasses, serial={}", serial);
    
    let mut fusion = FusionState::new(glasses);

    loop {
        let event = fusion.glasses.read_event().unwrap();
        println!("Event: {:#?}", event);
    }
}
