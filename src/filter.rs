//! Conditioning of raw tracker orientations: re-centring, drift correction
//! and smoothing.

use crate::orientation::Quat;

/// Below this head speed (deg/s) the head counts as still.
const STILL_SPEED: f64 = 5.0;
/// Time constant of the head speed estimate, in seconds.
const SPEED_TIME_CONSTANT: f64 = 0.3;
/// How fast the drift estimate follows where the head rests, in seconds.
const DRIFT_TIME_CONSTANT: f64 = 10.0;
/// Fraction of each head movement by which the heading may be corrected.
const DRIFT_FOLLOW: f64 = 0.1;
/// The gyro bias is measured over stretches of this many seconds of stillness.
const BIAS_WINDOW: f64 = 4.0;
/// Weight of each new measurement in the bias estimate.
const BIAS_UPDATE: f64 = 0.5;
/// Slides faster than this (deg/s) are taken as head movement, not bias.
const MAX_BIAS: f64 = 0.5;

pub struct HeadFilter {
    /// Heading that counts as "facing forward".
    reference: Quat,
    smoothed: Option<Quat>,
    /// Exponential smoothing time constant in seconds; 0 disables smoothing.
    time_constant: f64,
    recenter_pending: bool,
    /// Resting headings within this many degrees of front pull front towards
    /// them (`auto_recenter`); 0 disables it.
    drift_window: f64,
    /// Heading correction learnt but not yet applied, in degrees.
    drift: f64,
    /// Smoothed head speed in deg/s.
    speed: f64,
    /// Estimated gyro bias: how fast the heading slides by itself, in deg/s.
    bias: f64,
    /// Current stretch of stillness: its length in seconds and the heading
    /// change over it in degrees.
    still_time: f64,
    still_turn: f64,
    last_raw: Option<Quat>,
}

impl HeadFilter {
    /// The first orientation received becomes the forward direction.
    pub fn new(time_constant: f64, drift_window: f64) -> Self {
        HeadFilter {
            reference: Quat::IDENTITY,
            smoothed: None,
            time_constant,
            recenter_pending: true,
            drift_window,
            drift: 0.0,
            speed: 0.0,
            bias: 0.0,
            still_time: 0.0,
            still_turn: 0.0,
            last_raw: None,
        }
    }

    /// Make the next orientation received the forward direction.
    ///
    /// Only the heading is reset: trackers get pitch and roll from gravity, so
    /// those do not drift, while the heading does.
    pub fn recenter(&mut self) {
        self.recenter_pending = true;
    }

    /// Forget the smoothed orientation: the next one is taken as it is,
    /// instead of gliding there from where tracking left off.
    pub fn reset(&mut self) {
        self.smoothed = None;
        self.last_raw = None;
    }

    /// Feed a raw orientation, `dt` seconds after the previous one.
    pub fn update(&mut self, raw: Quat, dt: f64) -> Quat {
        if std::mem::take(&mut self.recenter_pending) {
            self.reference = Quat::from_euler_deg(raw.yaw_deg(), 0.0, 0.0);
            self.drift = 0.0;
        }
        self.correct_drift(raw, dt);
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

    /// The drift correction's state, for `--verbose`.
    pub fn describe(&self) -> String {
        format!(
            "speed {:4.1} deg/s, still {:3.1} s, bias {:+.3} deg/s, pending {:+5.1} deg",
            self.speed, self.still_time, self.bias, self.drift
        )
    }

    /// Automatic re-centring against the tracker's heading drift. Worn, the
    /// Maxwell's heading slides at up to ~0.2 deg/s even with the head still
    /// (gyro bias), and it loses ~0.5-1% of every turn, either way.
    ///
    /// The slide is measured while the head is still and cancelled all the
    /// time. On top of that, while the head rests near front, the heading it
    /// rests at is learnt slowly, so front becomes where you usually look and
    /// glances elsewhere hardly count. That correction is only applied while
    /// the head turns, a fraction of the movement at a time: then it is masked
    /// by the movement and adds no speaker updates (clicks) of its own.
    fn correct_drift(&mut self, raw: Quat, dt: f64) {
        let last = self.last_raw.replace(raw);
        if self.drift_window <= 0.0 || dt <= 0.0 {
            return;
        }
        let Some(last) = last else { return };
        let motion = last.angle_to_deg(raw);
        self.speed += (motion / dt - self.speed) * (1.0 - (-dt / SPEED_TIME_CONSTANT).exp());
        let mut step = self.bias * dt;
        if self.speed < STILL_SPEED {
            self.measure_bias(raw, last, dt);
            let heading = self.reference.conjugate().mul(raw).yaw_deg();
            if heading.abs() < self.drift_window {
                self.drift += (heading - self.drift) * (1.0 - (-dt / DRIFT_TIME_CONSTANT).exp());
            }
        } else {
            self.still_time = 0.0;
            self.still_turn = 0.0;
            let correction = (DRIFT_FOLLOW * motion).min(self.drift.abs()).copysign(self.drift);
            self.drift -= correction;
            step += correction;
        }
        self.reference = self.reference.mul(Quat::from_euler_deg(step, 0.0, 0.0));
    }

    /// Over a stretch of stillness small head movements average out, the
    /// bias doesn't. A steady slow turn is told apart by being faster.
    fn measure_bias(&mut self, raw: Quat, last: Quat, dt: f64) {
        self.still_time += dt;
        self.still_turn += (raw.yaw_deg() - last.yaw_deg() + 540.0).rem_euclid(360.0) - 180.0;
        if self.still_time < BIAS_WINDOW {
            return;
        }
        let rate = self.still_turn / self.still_time;
        if rate.abs() <= MAX_BIAS {
            self.bias += (rate - self.bias) * BIAS_UPDATE;
        }
        self.still_time = 0.0;
        self.still_turn = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_sample_becomes_forward() {
        let mut f = HeadFilter::new(0.0, 0.0);
        let q = f.update(Quat::from_euler_deg(120.0, 10.0, 0.0), 0.01);
        assert!(q.yaw_deg().abs() < 1e-9);
    }

    #[test]
    fn recenter_keeps_pitch() {
        let mut f = HeadFilter::new(0.0, 0.0);
        f.update(Quat::IDENTITY, 0.01);
        f.recenter();
        let q = f.update(Quat::from_euler_deg(90.0, 20.0, 0.0), 0.01);
        let expected = Quat::from_euler_deg(0.0, 20.0, 0.0);
        let dot = q.w * expected.w + q.x * expected.x + q.y * expected.y + q.z * expected.z;
        assert!((dot.abs() - 1.0).abs() < 1e-9, "{q:?}");
    }

    #[test]
    fn smoothing_converges() {
        let mut f = HeadFilter::new(0.05, 0.0);
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

    /// Heading after holding `yaw` still for `seconds`, at 30 Hz.
    fn hold(f: &mut HeadFilter, yaw: f64, seconds: f64) -> f64 {
        let mut out = 0.0;
        for _ in 0..(seconds * 30.0) as usize {
            out = f.update(Quat::from_euler_deg(yaw, 0.0, 0.0), 1.0 / 30.0).yaw_deg();
        }
        out
    }

    /// Turn from `from` to `to` at 90 deg/s.
    fn sweep(f: &mut HeadFilter, from: f64, to: f64) {
        let steps = ((to - from).abs() / 3.0).ceil() as usize;
        for i in 0..=steps {
            let yaw = from + (to - from) * i as f64 / steps as f64;
            f.update(Quat::from_euler_deg(yaw, 0.0, 0.0), 1.0 / 30.0);
        }
    }

    #[test]
    fn drift_is_corrected_while_turning_only() {
        let mut f = HeadFilter::new(0.0, 15.0);
        hold(&mut f, 0.0, 1.0);
        // The tracker has drifted: looking at the screen now reads 8 deg.
        let still = hold(&mut f, 8.0, 60.0);
        assert!((still - 8.0).abs() < 1e-9, "moved while still: {still}");
        for _ in 0..5 {
            sweep(&mut f, 8.0, 68.0);
            sweep(&mut f, 68.0, 8.0);
        }
        let yaw = hold(&mut f, 8.0, 0.1);
        assert!(yaw.abs() < 0.5, "{yaw}");
    }

    #[test]
    fn glances_hardly_move_front() {
        let mut f = HeadFilter::new(0.0, 15.0);
        hold(&mut f, 0.0, 30.0);
        // Two seconds at a screen corner, then back to the middle.
        sweep(&mut f, 0.0, 12.0);
        hold(&mut f, 12.0, 2.0);
        sweep(&mut f, 12.0, 0.0);
        let yaw = hold(&mut f, 0.0, 0.1);
        assert!(yaw.abs() < 3.0, "{yaw}");
    }

    #[test]
    fn looking_far_away_is_not_drift() {
        let mut f = HeadFilter::new(0.0, 15.0);
        hold(&mut f, 0.0, 1.0);
        hold(&mut f, 40.0, 60.0);
        sweep(&mut f, 40.0, 0.0);
        assert!(hold(&mut f, 0.0, 0.1).abs() < 1e-9);
    }

    /// Recorded 2026-10-02: worn and still, the heading slid at -0.2 deg/s.
    #[test]
    fn gyro_bias_is_cancelled() {
        let mut f = HeadFilter::new(0.0, 15.0);
        let mut seed = 1u32;
        let mut at = |t: f64| {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let jitter = (seed >> 8) as f64 / (1 << 24) as f64 * 0.2 - 0.1;
            Quat::from_euler_deg(-0.2 * t + jitter, 0.0, 0.0)
        };
        let mut yaw = |f: &mut HeadFilter, from: f64, to: f64| {
            let mut out = 0.0;
            for i in (from * 30.0) as usize..(to * 30.0) as usize {
                out = f.update(at(i as f64 / 30.0), 1.0 / 30.0).yaw_deg();
            }
            out
        };
        let after_20s = yaw(&mut f, 0.0, 20.0);
        let after_60s = yaw(&mut f, 20.0, 60.0);
        assert!(after_20s.abs() < 2.0, "{after_20s}");
        assert!((after_60s - after_20s).abs() < 0.3, "still sliding: {after_20s} -> {after_60s}");
    }

    #[test]
    fn slow_turn_is_not_bias() {
        let mut f = HeadFilter::new(0.0, 15.0);
        hold(&mut f, 0.0, 10.0);
        // 30 deg over 10 s, below the "moving" speed.
        for i in 0..300 {
            f.update(Quat::from_euler_deg(i as f64 * 0.1, 0.0, 0.0), 1.0 / 30.0);
        }
        let yaw = hold(&mut f, 30.0, 10.0);
        assert!((yaw - 30.0).abs() < 1e-6, "{yaw}");
    }
}

