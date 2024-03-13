use crate::{ARGlasses, DisplayMode, Error, GlassesEvent, Side};
use nalgebra::Isometry3;
use std::time::{SystemTime, UNIX_EPOCH};

type Result<T> = std::result::Result<T, Error>;

pub struct Dummy {}

impl Dummy {}

impl ARGlasses for Dummy {
    fn serial(&mut self) -> crate::Result<String> {
        Ok(String::from("dummy!"))
    }

    fn read_event(&mut self) -> crate::Result<GlassesEvent> {
        let result = GlassesEvent::AccGyro {
            accelerometer: Default::default(),
            gyroscope: Default::default(),
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs(),
        };
        Ok(result)
    }

    fn get_display_mode(&mut self) -> crate::Result<DisplayMode> {
        Ok(DisplayMode::SameOnBoth)
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> crate::Result<()> {
        Ok(())
    }

    fn display_fov(&self) -> f32 {
        24.0f32.to_radians()
    }

    fn imu_to_display_matrix(&self, side: Side, ipd: f32) -> Isometry3<f64> {
        todo!()
    }

    fn name(&self) -> &'static str {
        "dummy"
    }

    fn display_delay(&self) -> u64 {
        0
    }
}
