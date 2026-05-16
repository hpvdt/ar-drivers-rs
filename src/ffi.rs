use crate::connection::Connection;
use crate::Fusion;

/// Euler-angle FFI result.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct EulerResult {
    /// 1 when the read succeeds, 0 otherwise.
    pub success: i32,
    /// First Euler component in degrees.
    pub x: f32,
    /// Second Euler component in degrees.
    pub y: f32,
    /// Third Euler component in degrees.
    pub z: f32,
}

impl EulerResult {
    fn failed() -> Self {
        Self {
            success: 0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

/// Quaternion FFI result, ordered as w, i, j, k.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct QuaternionResult {
    /// 1 when the read succeeds, 0 otherwise.
    pub success: i32,
    /// Scalar component.
    pub w: f32,
    /// First vector component.
    pub i: f32,
    /// Second vector component.
    pub j: f32,
    /// Third vector component.
    pub k: f32,
}

impl QuaternionResult {
    fn failed() -> Self {
        Self {
            success: 0,
            w: 0.0,
            i: 0.0,
            j: 0.0,
            k: 0.0,
        }
    }
}

#[no_mangle]
pub extern "C" fn StartConnection() -> i32 {
    Connection::start().unwrap();
    // println!("connection started");
    1
    // .map_or_else(|_| 1, |_| 0)
}

#[no_mangle]
pub extern "C" fn StopConnection() -> i32 {
    Connection::stop().unwrap();
    // println!("connection stopped");
    1
    // .map_or_else(|_| 1, |_| 0)
}

#[no_mangle]
pub extern "C" fn GetEuler() -> EulerResult {
    let Ok(euler) = Connection::read_fusion(&|ff| ff.attitude_euler_deg()) else {
        return EulerResult::failed();
    };

    EulerResult {
        success: 1,
        x: euler.x,
        y: euler.y,
        z: euler.z,
    }
}

#[no_mangle]
pub extern "C" fn GetQuaternion() -> QuaternionResult {
    let Ok(quaternion) = Connection::read_fusion(&|ff| ff.attitude_quaternion()) else {
        return QuaternionResult::failed();
    };

    let ijkw = quaternion.as_vector();
    QuaternionResult {
        success: 1,
        w: ijkw[3],
        i: ijkw[0],
        j: ijkw[1],
        k: ijkw[2],
    }
}
