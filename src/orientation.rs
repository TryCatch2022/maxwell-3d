//! Head orientation maths.
//!
//! Frame (same as SOFA): x = forward, y = left, z = up. Angles in degrees;
//! azimuth is counter-clockwise from the front, so positive yaw means the
//! head turns left.

/// Unit quaternion describing the head orientation relative to the room.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quat {
    pub const IDENTITY: Quat = Quat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    /// Intrinsic Z-Y-X rotation: yaw about up, pitch about left (positive
    /// looks down), roll about forward (positive tilts the right ear down).
    pub fn from_euler_deg(yaw: f64, pitch: f64, roll: f64) -> Quat {
        let (sy, cy) = (yaw.to_radians() / 2.0).sin_cos();
        let (sp, cp) = (pitch.to_radians() / 2.0).sin_cos();
        let (sr, cr) = (roll.to_radians() / 2.0).sin_cos();
        Quat {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    pub fn conjugate(self) -> Quat {
        Quat { w: self.w, x: -self.x, y: -self.y, z: -self.z }
    }

    /// Hamilton product: apply `rhs` first, then `self`.
    pub fn mul(self, rhs: Quat) -> Quat {
        Quat {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }

    pub fn normalize(self) -> Quat {
        let n = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        Quat { w: self.w / n, x: self.x / n, y: self.y / n, z: self.z / n }
    }

    /// Spherical interpolation from `self` (t = 0) to `to` (t = 1).
    pub fn slerp(self, to: Quat, t: f64) -> Quat {
        let mut dot = self.w * to.w + self.x * to.x + self.y * to.y + self.z * to.z;
        // q and -q are the same rotation: take the short way round.
        let to = if dot < 0.0 {
            dot = -dot;
            Quat { w: -to.w, x: -to.x, y: -to.y, z: -to.z }
        } else {
            to
        };
        let (a, b) = if dot > 0.9995 {
            (1.0 - t, t)
        } else {
            let theta = dot.acos();
            let s = theta.sin();
            (((1.0 - t) * theta).sin() / s, (t * theta).sin() / s)
        };
        Quat {
            w: a * self.w + b * to.w,
            x: a * self.x + b * to.x,
            y: a * self.y + b * to.y,
            z: a * self.z + b * to.z,
        }
        .normalize()
    }

    /// Angle of the rotation that takes `self` to `other`, in degrees.
    pub fn angle_to_deg(self, other: Quat) -> f64 {
        let dot = self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z;
        2.0 * dot.abs().min(1.0).acos().to_degrees()
    }

    /// Heading of the forward axis around the vertical, in degrees.
    pub fn yaw_deg(self) -> f64 {
        let f = self.rotate([1.0, 0.0, 0.0]);
        f[1].atan2(f[0]).to_degrees()
    }

    /// Rotate vector `v` by this quaternion.
    pub fn rotate(self, v: [f64; 3]) -> [f64; 3] {
        let u = [self.x, self.y, self.z];
        let t = scale(cross(u, v), 2.0);
        add(add(v, scale(t, self.w)), cross(u, t))
    }
}

/// A speaker direction as (azimuth, elevation) in degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Direction {
    pub azimuth: f64,
    pub elevation: f64,
}

impl Direction {
    pub fn to_vector(self) -> [f64; 3] {
        let (sa, ca) = self.azimuth.to_radians().sin_cos();
        let (se, ce) = self.elevation.to_radians().sin_cos();
        [ce * ca, ce * sa, se]
    }

    /// Azimuth is normalised to [0, 360) as the spatializer expects.
    pub fn from_vector(v: [f64; 3]) -> Direction {
        let azimuth = v[1].atan2(v[0]).to_degrees().rem_euclid(360.0);
        let elevation = v[2].clamp(-1.0, 1.0).asin().to_degrees();
        Direction { azimuth, elevation }
    }

    /// Where a speaker fixed in the room appears relative to the head.
    pub fn relative_to(self, head: Quat) -> Direction {
        Direction::from_vector(head.conjugate().rotate(self.to_vector()))
    }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_dir(d: Direction, azimuth: f64, elevation: f64) {
        let da = (d.azimuth - azimuth + 180.0).rem_euclid(360.0) - 180.0;
        assert!(da.abs() < 1e-9 && (d.elevation - elevation).abs() < 1e-9, "{d:?}");
    }

    fn assert_quat(a: Quat, b: Quat) {
        let dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        assert!((dot.abs() - 1.0).abs() < 1e-9, "{a:?} != {b:?}");
    }

    #[test]
    fn mul_composes_yaws() {
        let q = Quat::from_euler_deg(30.0, 0.0, 0.0).mul(Quat::from_euler_deg(40.0, 0.0, 0.0));
        assert_quat(q, Quat::from_euler_deg(70.0, 0.0, 0.0));
    }

    #[test]
    fn slerp_halfway_takes_short_path() {
        let a = Quat::from_euler_deg(350.0, 0.0, 0.0);
        let b = Quat::from_euler_deg(10.0, 0.0, 0.0);
        assert_quat(a.slerp(b, 0.5), Quat::IDENTITY);
    }

    #[test]
    fn angle_between_poses() {
        let a = Quat::from_euler_deg(10.0, 0.0, 0.0);
        let b = Quat::from_euler_deg(350.0, 0.0, 0.0);
        assert!((a.angle_to_deg(b) - 20.0).abs() < 1e-9);
    }

    #[test]
    fn yaw_ignores_pitch() {
        let q = Quat::from_euler_deg(60.0, 20.0, 0.0);
        assert!((q.yaw_deg() - 60.0).abs() < 1e-9);
    }

    #[test]
    fn identity_keeps_direction() {
        let d = Direction { azimuth: 30.0, elevation: 0.0 };
        assert_dir(d.relative_to(Quat::IDENTITY), 30.0, 0.0);
    }

    #[test]
    fn turning_left_moves_front_to_the_right() {
        let head = Quat::from_euler_deg(90.0, 0.0, 0.0);
        let front = Direction { azimuth: 0.0, elevation: 0.0 };
        assert_dir(front.relative_to(head), 270.0, 0.0);
    }

    #[test]
    fn looking_down_raises_front() {
        let head = Quat::from_euler_deg(0.0, 30.0, 0.0);
        let front = Direction { azimuth: 0.0, elevation: 0.0 };
        assert_dir(front.relative_to(head), 0.0, 30.0);
    }

    #[test]
    fn rolling_right_lowers_left_side() {
        // Right ear down: the left ear points at the ceiling, so a speaker
        // on the left at ear height drops below the ear line.
        let head = Quat::from_euler_deg(0.0, 0.0, 45.0);
        let left = Direction { azimuth: 90.0, elevation: 0.0 };
        assert_dir(left.relative_to(head), 90.0, -45.0);
    }
}
