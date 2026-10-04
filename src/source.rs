//! Head orientation sources: the Maxwell hidraw stream, a backup tracker,
//! or synthetic motion for testing.

use std::fs::{self, File, OpenOptions};
use std::io::{self, Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use crate::orientation::Quat;

pub trait OrientationSource {
    /// Latest head orientation, or `None` if nothing new is available.
    fn poll(&mut self) -> Option<Quat>;

    /// Whether orientations are coming in. When it turns true again the
    /// tracker may have lost its frame, and the head is re-centred.
    fn connected(&self) -> bool {
        true
    }
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

/// Head tracking from the Audeze Maxwell over its vendor HID interface, either
/// on the headset itself (USB-C) or through the wireless dongle.
///
/// Learned from USB captures of the Windows apps: writing 0x3c then 0xfc to
/// register 0x092e starts a stream of input report 0x0A at ~60 Hz, four
/// little-endian f32 (x, y, z, w) forming the head-to-world rotation in the
/// headset's frame (x down, y left, z forward); writing 0x3c stops it.
/// Through the dongle the commands carry a flag asking it to forward them.
///
/// Survives the headset or dongle coming and going: a node that disappears is
/// looked for again, and when the stream stops (headset off or out of range,
/// the dongle stays) tracking is switched on again until it resumes.
pub struct MaxwellHid {
    /// Node given with --hidraw; otherwise the best one present is used.
    fixed: Option<PathBuf>,
    device: Option<Device>,
    /// When to look for a node, or re-enable a silent stream, next.
    next_attempt: Instant,
    last_report: Option<Instant>,
    /// First orientation of the stream: the head pose that counts as "level, facing front".
    reference: Option<Quat>,
    /// Last problem logged, so that retries don't repeat it.
    last_problem: Option<String>,
}

/// The stream runs at ~60 Hz even when the head is still: this long without a
/// report means the headset is off, out of range or has stopped streaming.
const STALE: Duration = Duration::from_secs(2);
/// How often to look for a node, or try to restart a stale stream.
const RETRY: Duration = Duration::from_secs(3);

/// HID IDs and the routing byte of the commands: 0x80 makes the dongle
/// forward them to the headset (it rejects them otherwise).
const AUDEZE_DEVICES: [(&str, u8); 2] = [("00003329:00004BAA", 0x00), ("00003329:00004BA9", 0x80)];
/// Head tracking register; bits 0xc0 switch the report 0x0A stream on.
const TRACKING_ON: [u8; 2] = [0x3c, 0xfc];
const TRACKING_OFF: u8 = 0x3c;

/// The routing byte for a hidraw node such as `hidraw3`, if it is a Maxwell.
fn maxwell_route(node: &std::ffi::OsStr) -> Option<u8> {
    let uevent = fs::read_to_string(Path::new("/sys/class/hidraw").join(node).join("device/uevent")).ok()?;
    let id = uevent.lines().find_map(|l| l.strip_prefix("HID_ID=0003:"))?;
    AUDEZE_DEVICES.iter().find(|(known, _)| *known == id).map(|&(_, route)| route)
}

/// Whether the USB device behind a hidraw node also has audio interfaces.
/// The headset on USB-C only does (and only streams head tracking there) in
/// USB audio mode; while it is connected to the dongle it is HID-only.
fn has_usb_audio(node: &std::ffi::OsStr) -> bool {
    let Ok(hid) = fs::canonicalize(Path::new("/sys/class/hidraw").join(node).join("device")) else {
        return false;
    };
    // .../<usb device>/<interface>/<hid device>
    let Some(usb_device) = hid.parent().and_then(Path::parent) else { return false };
    let Ok(entries) = fs::read_dir(usb_device) else { return false };
    entries.flatten().any(|e| {
        fs::read_to_string(e.path().join("bInterfaceClass")).is_ok_and(|class| class.trim() == "01")
    })
}

impl MaxwellHid {
    pub fn new(fixed: Option<PathBuf>) -> Self {
        MaxwellHid {
            fixed,
            device: None,
            next_attempt: Instant::now(),
            last_report: None,
            reference: None,
            last_problem: None,
        }
    }

    /// The hidraw node to use: the headset if it is on USB-C in USB audio
    /// mode, else the dongle, else the headset anyway.
    pub fn find() -> Option<PathBuf> {
        let mut nodes: Vec<_> = fs::read_dir("/sys/class/hidraw").ok()?.flatten().map(|e| e.file_name()).collect();
        nodes.sort();
        let rank = |node: &std::ffi::OsStr| match maxwell_route(node)? {
            0x00 if has_usb_audio(node) => Some(0),
            0x00 => Some(2),
            _ => Some(1),
        };
        let best = nodes.iter().filter_map(|node| Some((rank(node)?, node))).min_by_key(|&(rank, _)| rank)?;
        Some(Path::new("/dev").join(best.1))
    }

    /// Log `message` unless it is the one logged last.
    fn problem(&mut self, message: String) {
        if self.last_problem.as_ref() != Some(&message) {
            eprintln!("{message}");
            self.last_problem = Some(message);
        }
    }

    fn connect(&mut self) {
        let Some(path) = self.fixed.clone().or_else(MaxwellHid::find) else {
            self.problem("no Audeze Maxwell headset or dongle found, waiting for one".into());
            return;
        };
        let mut device = match Device::open(&path) {
            Ok(device) => device,
            Err(e) if e.kind() == io::ErrorKind::PermissionDenied => {
                self.problem(format!(
                    "cannot open {}: {e} (is udev/70-audeze-maxwell.rules installed?)",
                    path.display()
                ));
                return;
            }
            Err(e) => {
                self.problem(format!("cannot open {}: {e}", path.display()));
                return;
            }
        };
        eprintln!("using {}", path.display());
        self.last_problem = None;
        if let Err(e) = device.start_tracking() {
            // Kept anyway: retried while no data comes.
            self.problem(format!("cannot switch head tracking on through {}: {e}", path.display()));
        }
        self.device = Some(device);
        self.last_report = None;
    }

    /// Called while the stream is silent: move to a better node if one
    /// appeared (e.g. the headset was plugged in), else ask for the stream again.
    fn restart_stream(&mut self) {
        let Some(device) = &mut self.device else { return };
        if self.fixed.is_none() && MaxwellHid::find().is_some_and(|best| best != device.path) {
            self.device = None;
            self.connect();
            return;
        }
        let path = device.path.display().to_string();
        let result = device.start_tracking();
        match result {
            Ok(()) => self.problem(format!("no head tracking data from {path} (headset off or out of range?)")),
            Err(e) => self.problem(format!("cannot switch head tracking on through {path}: {e}")),
        }
    }
}

/// An open hidraw node of a headset or dongle.
struct Device {
    file: File,
    path: PathBuf,
    route: u8,
    /// Cleared once the node has failed, so nothing more is written to it.
    present: bool,
}

impl Device {
    fn open(path: &Path) -> io::Result<Self> {
        let route = path.file_name().and_then(maxwell_route).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "not an Audeze Maxwell headset or dongle")
        })?;
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(path)?;
        Ok(Device { file, path: path.to_owned(), route, present: true })
    }

    fn start_tracking(&mut self) -> io::Result<()> {
        for value in TRACKING_ON {
            self.write_register(value)?;
            std::thread::sleep(Duration::from_millis(100));
        }
        Ok(())
    }

    fn write_register(&mut self, value: u8) -> io::Result<()> {
        self.file.write_all(&register_write_report(self.route, value))
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        if !self.present {
            return;
        }
        if let Err(e) = self.write_register(TRACKING_OFF) {
            eprintln!("could not switch Maxwell head tracking off: {e}");
        }
    }
}

/// Output report 0x06: [0x06, 4 + len, route, 0x05, 0x5a, len (LE16), payload],
/// with the write payload being [0x00, 0x09, register, 0x00, value].
fn register_write_report(route: u8, value: u8) -> [u8; 62] {
    let mut report = [0u8; 62];
    report[..12].copy_from_slice(&[0x06, 0x09, route, 0x05, 0x5a, 0x05, 0x00, 0x00, 0x09, 0x2e, 0x00, value]);
    report
}

impl OrientationSource for MaxwellHid {
    fn poll(&mut self) -> Option<Quat> {
        let now = Instant::now();
        let was_connected = self.connected();
        let Some(device) = &mut self.device else {
            if now >= self.next_attempt {
                self.next_attempt = now + RETRY;
                self.connect();
            }
            return None;
        };
        let mut latest = None;
        let mut buf = [0u8; 64];
        loop {
            match device.file.read(&mut buf) {
                Ok(0) => break,
                Ok(len) => latest = decode_tracking_report(&buf[..len]).or(latest),
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    // Unplugged: look for a node again right away, the other
                    // one (headset or dongle) may still be there.
                    device.present = false;
                    let message = format!("lost {}: {e}", device.path.display());
                    self.device = None;
                    self.last_report = None;
                    self.next_attempt = now;
                    self.problem(message);
                    return None;
                }
            }
        }
        let Some(q) = latest else {
            if !was_connected && now >= self.next_attempt {
                self.next_attempt = now + RETRY;
                self.restart_stream();
            }
            return None;
        };
        if !was_connected {
            // A new stream: the headset may have been off, its frame is not the old one.
            eprintln!("head tracking data coming from {}", device.path.display());
            self.reference = None;
            self.last_problem = None;
        }
        self.last_report = Some(now);
        self.next_attempt = now + STALE;
        Some(headset_to_head(*self.reference.get_or_insert(q), q))
    }

    fn connected(&self) -> bool {
        self.last_report.is_some_and(|t| t.elapsed() < STALE)
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

    /// The enable writes as the Windows apps send them, on USB-C and through the dongle.
    #[test]
    fn register_writes_match_capture() {
        assert_eq!(register_write_report(0x00, 0xfc)[..12], [6, 9, 0, 5, 0x5a, 5, 0, 0, 9, 0x2e, 0, 0xfc]);
        assert_eq!(register_write_report(0x80, 0x3c)[..12], [6, 9, 0x80, 5, 0x5a, 5, 0, 0, 9, 0x2e, 0, 0x3c]);
    }

    #[test]
    fn rejects_other_reports() {
        assert!(decode_tracking_report(&[0x0c, 0x01]).is_none());
    }
}
