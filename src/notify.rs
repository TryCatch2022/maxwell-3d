//! Status reports to systemd when started as a `Type=notify` service
//! (sd_notify protocol); without `$NOTIFY_SOCKET` they go nowhere.

use std::cell::{Cell, RefCell};
use std::os::linux::net::SocketAddrExt;
use std::os::unix::ffi::OsStrExt;
use std::os::unix::net::{SocketAddr, UnixDatagram};
use std::time::{Duration, Instant};

pub struct Notifier {
    socket: Option<(UnixDatagram, SocketAddr)>,
    status: RefCell<String>,
    last_ping: Cell<Option<Instant>>,
}

/// Well inside the service's `WatchdogSec`.
const PING_INTERVAL: Duration = Duration::from_secs(1);

impl Notifier {
    pub fn from_env() -> Self {
        let socket = std::env::var_os("NOTIFY_SOCKET").and_then(|path| {
            let addr = match path.as_bytes().strip_prefix(b"@") {
                Some(name) => SocketAddr::from_abstract_name(name).ok()?,
                None => SocketAddr::from_pathname(&path).ok()?,
            };
            Some((UnixDatagram::unbound().ok()?, addr))
        });
        Notifier { socket, status: RefCell::default(), last_ping: Cell::new(None) }
    }

    pub fn send(&self, state: &str) {
        if let Some((socket, addr)) = &self.socket {
            let _ = socket.send_to_addr(state.as_bytes(), addr);
        }
    }

    /// The line `systemctl status` shows; only sent when it changes, which is returned.
    pub fn status(&self, status: &str) -> bool {
        if *self.status.borrow() == status {
            return false;
        }
        self.send(&format!("STATUS={status}"));
        *self.status.borrow_mut() = status.to_owned();
        true
    }

    /// Tell the watchdog the main loop is alive; call it often, it rate-limits.
    pub fn ping(&self) {
        if self.last_ping.get().is_none_or(|t| t.elapsed() >= PING_INTERVAL) {
            self.send("WATCHDOG=1");
            self.last_ping.set(Some(Instant::now()));
        }
    }
}
