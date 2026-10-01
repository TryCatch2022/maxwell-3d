//! Head orientation sources: the Maxwell hidraw stream, a backup tracker,
//! or synthetic motion for testing.

use std::fs::{self, File, OpenOptions};
use std::io::{self, Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};
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

/// Receives OpenTrack's "UDP over network" output: six little-endian f64
/// (x, y, z in cm, then yaw, pitch, roll in degrees) per datagram.
///
/// Angles are taken in maxwell3d's convention (positive yaw turns left,
/// positive pitch looks down, positive roll tilts the right ear down); use
/// OpenTrack's per-axis invert options if its output disagrees.
pub struct OpenTrackUdp {
    socket: UdpSocket,
}

impl OpenTrackUdp {
    pub fn bind(addr: SocketAddr) -> io::Result<Self> {
        let socket = UdpSocket::bind(addr)?;
        socket.set_nonblocking(true)?;
        Ok(OpenTrackUdp { socket })
    }
}

impl OrientationSource for OpenTrackUdp {
    fn poll(&mut self) -> Option<Quat> {
        // Drain the socket and keep only the most recent pose.
        let mut latest = None;
        let mut buf = [0u8; 64];
        while let Ok(len) = self.socket.recv(&mut buf) {
            if len != 48 {
                continue;
            }
            let value = |i: usize| f64::from_le_bytes(buf[i * 8..i * 8 + 8].try_into().unwrap());
            let (yaw, pitch, roll) = (value(3), value(4), value(5));
            if yaw.is_finite() && pitch.is_finite() && roll.is_finite() {
                latest = Some(Quat::from_euler_deg(yaw, pitch, roll));
            }
        }
        latest
    }
}

/// Head tracking from the Audeze Maxwell over its vendor HID interface.
///
/// Learned from a USB capture of the Windows apps (headset on USB-C, in USB
/// audio mode): writing 0x3c then 0xfc to register 0x092e starts a stream of
/// input report 0x0A at ~60 Hz, four little-endian f32 (x, y, z, w) forming
/// the head-to-world rotation in the headset's frame (x down, y left,
/// z forward); writing 0x3c stops it.
pub struct MaxwellHid {
    file: File,
    /// First orientation of the stream: the head pose that counts as "level, facing front".
    reference: Option<Quat>,
}

const AUDEZE_HEADSET: &str = "00003329:00004BAA";
/// Head tracking register; bits 0xc0 switch the report 0x0A stream on.
const TRACKING_ON: [u8; 2] = [0x3c, 0xfc];
const TRACKING_OFF: u8 = 0x3c;

impl MaxwellHid {
    /// The hidraw node of the headset (USB-C), if one is plugged in.
    pub fn find() -> Option<PathBuf> {
        let mut nodes: Vec<_> = fs::read_dir("/sys/class/hidraw").ok()?.flatten().collect();
        nodes.sort_by_key(|e| e.file_name());
        nodes.into_iter().find_map(|entry| {
            let uevent = fs::read_to_string(entry.path().join("device/uevent")).ok()?;
            let is_headset = uevent.lines().any(|l| l == format!("HID_ID=0003:{AUDEZE_HEADSET}"));
            is_headset.then(|| Path::new("/dev").join(entry.file_name()))
        })
    }

    pub fn open(path: &Path) -> io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(path)?;
        let mut source = MaxwellHid { file, reference: None };
        for value in TRACKING_ON {
            source.write_register(value)?;
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        Ok(source)
    }

    /// Output report 0x06: [0x06, 4 + len, 0, 0x05, 0x5a, len (LE16), payload],
    /// with the write payload being [0x00, 0x09, register, 0x00, value].
    fn write_register(&mut self, value: u8) -> io::Result<()> {
        let mut report = [0u8; 62];
        report[..12].copy_from_slice(&[0x06, 0x09, 0x00, 0x05, 0x5a, 0x05, 0x00, 0x00, 0x09, 0x2e, 0x00, value]);
        self.file.write_all(&report)
    }
}

impl OrientationSource for MaxwellHid {
    fn poll(&mut self) -> Option<Quat> {
        let mut latest = None;
        let mut buf = [0u8; 64];
        while let Ok(len) = self.file.read(&mut buf) {
            latest = decode_tracking_report(&buf[..len]).or(latest);
        }
        let q = latest?;
        Some(headset_to_head(*self.reference.get_or_insert(q), q))
    }
}

/// Report 0x0A: four little-endian f32 (x, y, z, w).
fn decode_tracking_report(report: &[u8]) -> Option<Quat> {
    if report.len() != 17 || report[0] != 0x0a {
        return None;
    }
    let value = |i: usize| f32::from_le_bytes(report[1 + i * 4..5 + i * 4].try_into().unwrap()) as f64;
    let q = Quat { w: value(3), x: value(0), y: value(1), z: value(2) };
    (q.w.is_finite() && q.x.is_finite() && q.y.is_finite() && q.z.is_finite()).then(|| q.normalize())
}

/// Rotation from `reference` to `q`, in the headset's body frame, with the
/// axes renamed into ours: forward = z, left = y, up = -x.
fn headset_to_head(reference: Quat, q: Quat) -> Quat {
    let r = reference.conjugate().mul(q);
    Quat { w: r.w, x: r.z, y: r.y, z: -r.x }
}

impl Drop for MaxwellHid {
    fn drop(&mut self) {
        if let Err(e) = self.write_register(TRACKING_OFF) {
            eprintln!("could not switch Maxwell head tracking off: {e}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn report(hex: &str) -> Quat {
        let bytes: Vec<u8> =
            (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()).collect();
        decode_tracking_report(&bytes).unwrap()
    }

    /// Reports captured while the wearer looked ahead, then turned left, looked
    /// up and looked down (angles as seen in the capture).
    #[test]
    fn captured_reports_give_head_motion() {
        let ahead = report("0a510e71bf8826e83d1e7c54bea867753e");
        let left = headset_to_head(ahead, report("0a73f36a3feb0f3bbebe100d3ebf2ba63e"));
        let up = headset_to_head(ahead, report("0a4825733ff87b86bd26262dbe6c8882be"));
        let down = headset_to_head(ahead, report("0a64bb2cbfdf28803e777e2bbf4de53a3e"));
        assert!((left.yaw_deg() - 66.0).abs() < 2.0, "left: {}", left.yaw_deg());
        let pitch = |q: Quat| -q.rotate([1.0, 0.0, 0.0])[2].asin().to_degrees();
        assert!((pitch(up) + 43.0).abs() < 2.0, "up: {}", pitch(up));
        assert!((pitch(down) - 64.0).abs() < 2.0, "down: {}", pitch(down));
    }

    #[test]
    fn rejects_other_reports() {
        assert!(decode_tracking_report(&[0x0c, 0x01]).is_none());
    }
}
