//! Conditioning of raw tracker orientations: re-centring and smoothing.

use crate::orientation::Quat;

pub struct HeadFilter {
    /// Heading that counts as "facing forward".
    reference: Quat,
    smoothed: Option<Quat>,
    /// Exponential smoothing time constant in seconds; 0 disables smoothing.
    time_constant: f64,
    recenter_pending: bool,
}

impl HeadFilter {
    /// The first orientation received becomes the forward direction.
    pub fn new(time_constant: f64) -> Self {
        HeadFilter { reference: Quat::IDENTITY, smoothed: None, time_constant, recenter_pending: true }
    }

    /// Make the next orientation received the forward direction.
    ///
    /// Only the heading is reset: trackers get pitch and roll from gravity, so
    /// those do not drift, while the heading does.
    pub fn recenter(&mut self) {
        self.recenter_pending = true;
    }

    /// Feed a raw orientation, `dt` seconds after the previous one.
    pub fn update(&mut self, raw: Quat, dt: f64) -> Quat {
        if std::mem::take(&mut self.recenter_pending) {
            self.reference = Quat::from_euler_deg(raw.yaw_deg(), 0.0, 0.0);
        }
        let target = self.reference.conjugate().mul(raw);
        let next = match self.smoothed {
            Some(prev) if self.time_constant > 0.0 => {
                prev.slerp(target, 1.0 - (-dt / self.time_constant).exp())
            }
            _ => target,
        };
        self.smoothed = Some(next);
        next
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_sample_becomes_forward() {
        let mut f = HeadFilter::new(0.0);
        let q = f.update(Quat::from_euler_deg(120.0, 10.0, 0.0), 0.01);
        assert!(q.yaw_deg().abs() < 1e-9);
    }

    #[test]
    fn recenter_keeps_pitch() {
        let mut f = HeadFilter::new(0.0);
        f.update(Quat::IDENTITY, 0.01);
        f.recenter();
        let q = f.update(Quat::from_euler_deg(90.0, 20.0, 0.0), 0.01);
        let expected = Quat::from_euler_deg(0.0, 20.0, 0.0);
        let dot = q.w * expected.w + q.x * expected.x + q.y * expected.y + q.z * expected.z;
        assert!((dot.abs() - 1.0).abs() < 1e-9, "{q:?}");
    }

    #[test]
    fn smoothing_converges() {
        let mut f = HeadFilter::new(0.05);
        f.update(Quat::IDENTITY, 0.01);
        let target = Quat::from_euler_deg(60.0, 0.0, 0.0);
        let first = f.update(target, 0.01).yaw_deg();
        assert!(first > 0.0 && first < 60.0, "{first}");
        let mut last = first;
        for _ in 0..100 {
            last = f.update(target, 0.01).yaw_deg();
        }
        assert!((last - 60.0).abs() < 1e-3, "{last}");
    }
}
