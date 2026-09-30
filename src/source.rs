//! Head orientation sources: the Maxwell hidraw stream, a backup tracker,
//! or synthetic motion for testing.

use std::time::Instant;

use crate::orientation::Quat;

pub trait OrientationSource {
    /// Latest head orientation, or `None` if nothing new is available.
    fn poll(&mut self) -> Option<Quat>;
}

/// Turns the head slowly around the vertical axis, forever.
pub struct FakeRotation {
    start: Instant,
    degrees_per_second: f64,
}

impl FakeRotation {
    pub fn new(degrees_per_second: f64) -> Self {
        FakeRotation { start: Instant::now(), degrees_per_second }
    }
}

impl OrientationSource for FakeRotation {
    fn poll(&mut self) -> Option<Quat> {
        let yaw = self.start.elapsed().as_secs_f64() * self.degrees_per_second;
        Some(Quat::from_euler_deg(yaw.rem_euclid(360.0), 0.0, 0.0))
    }
}
