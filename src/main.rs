mod filter;
mod notify;
mod orientation;
mod source;

use std::cell::{Cell, RefCell};
use std::fs::{File, OpenOptions};
use std::os::fd::AsRawFd;
use std::path::PathBuf;
use std::rc::Rc;
use std::time::{Duration, Instant};

use pipewire as pw;
use pw::spa::param::ParamType;
use pw::spa::pod::{Object, Pod, Property, Value, serialize::PodSerializer};
use pw::loop_::Signal;
use pw::types::ObjectType;

use filter::HeadFilter;
use notify::Notifier;
use orientation::{Direction, Quat};
use source::{FakeRotation, MaxwellHid, OpenTrackUdp, OrientationSource};

/// Spatializer nodes in `pipewire/maxwell-3d-sink.conf` and their room positions.
const SPEAKERS: [(&str, f64); 7] = [
    ("spFL", 30.0),
    ("spFR", 330.0),
    ("spFC", 0.0),
    ("spSL", 90.0),
    ("spSR", 270.0),
    ("spRL", 150.0),
    ("spRR", 210.0),
];

/// Exit codes a restart wouldn't fix (`RestartPreventExitStatus` in the service).
const EXIT_USAGE: i32 = 2;
const EXIT_ALREADY_RUNNING: i32 = 3;

/// How long to wait before connecting to PipeWire again.
const RECONNECT_DELAY: Duration = Duration::from_secs(2);

enum SourceKind {
    Fake,
    OpenTrack,
    Maxwell,
}

struct Args {
    node_name: String,
    source: SourceKind,
    listen: std::net::SocketAddr,
    hidraw: Option<PathBuf>,
    rate_hz: f64,
    speed: f64,
    smoothing_ms: f64,
    deadband_deg: f64,
    auto_recenter_deg: f64,
    verbose: bool,
}

fn parse_args() -> Args {
    let mut args = Args {
        node_name: "effect_input.maxwell_3d".into(),
        source: SourceKind::Maxwell,
        listen: ([127, 0, 0, 1], 4242).into(),
        hidraw: None,
        rate_hz: 30.0,
        speed: 20.0,
        smoothing_ms: 30.0,
        deadband_deg: 1.0,
        auto_recenter_deg: 15.0,
        verbose: false,
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut value = || it.next().unwrap_or_else(|| usage(&format!("{arg} needs a value")));
        // A number that must be at least zero (or above it, if `positive`).
        let number = |value: String, positive: bool| match value.parse::<f64>() {
            Ok(v) if v.is_finite() && (v > 0.0 || !positive && v == 0.0) => v,
            _ => usage(&format!("bad {arg} {value}")),
        };
        match arg.as_str() {
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            "-v" | "--verbose" => args.verbose = true,
            "--node" => args.node_name = value(),
            "--source" => {
                args.source = match value().as_str() {
                    "fake" => SourceKind::Fake,
                    "opentrack" => SourceKind::OpenTrack,
                    "maxwell" => SourceKind::Maxwell,
                    other => usage(&format!("unknown source {other}")),
                }
            }
            "--listen" => args.listen = value().parse().unwrap_or_else(|_| usage("bad --listen")),
            "--hidraw" => args.hidraw = Some(value().into()),
            "--rate" => args.rate_hz = number(value(), true),
            "--speed" => args.speed = value().parse().unwrap_or_else(|_| usage("bad --speed")),
            "--deadband" => args.deadband_deg = number(value(), false),
            "--smoothing" => args.smoothing_ms = number(value(), false),
            "--auto-recenter" => args.auto_recenter_deg = number(value(), false),
            _ => usage(&format!("unknown argument {arg}")),
        }
    }
    args
}

fn print_usage() {
    eprintln!(
        "usage: maxwell3d [--source maxwell|opentrack|fake] [--hidraw PATH] [--listen ADDR:PORT]\n\
         \x20                [--node NAME] [--rate HZ] [--speed DEG_PER_SEC]\n\
         \x20                [--smoothing MS] [--deadband DEG] [--auto-recenter DEG] [--verbose]\n\
         SIGUSR1 makes the current heading the new front, SIGUSR2 switches tracking off and on"
    );
}

fn usage(error: &str) -> ! {
    eprintln!("error: {error}");
    print_usage();
    std::process::exit(EXIT_USAGE);
}

/// Build a filter-chain `Props` pod: `params = [ "spFL:Azimuth" 30.0 ... ]`.
fn speaker_props(head: Quat) -> Vec<u8> {
    let mut params = Vec::with_capacity(SPEAKERS.len() * 4);
    for (name, azimuth) in SPEAKERS {
        let d = Direction { azimuth, elevation: 0.0 }.relative_to(head);
        params.push(Value::String(format!("{name}:Azimuth")));
        params.push(Value::Float(d.azimuth as f32));
        params.push(Value::String(format!("{name}:Elevation")));
        params.push(Value::Float(d.elevation as f32));
    }
    let object = Value::Object(Object {
        type_: pw::spa::sys::SPA_TYPE_OBJECT_Props,
        id: pw::spa::sys::SPA_PARAM_Props,
        properties: vec![Property::new(pw::spa::sys::SPA_PROP_params, Value::Struct(params))],
    });
    PodSerializer::serialize(std::io::Cursor::new(Vec::new()), &object)
        .expect("serialize Props pod")
        .0
        .into_inner()
}

/// Head tracking state, kept across PipeWire reconnections.
struct Tracker {
    source: Box<dyn OrientationSource>,
    filter: HeadFilter,
    /// Switched with SIGUSR2; while off the speakers stay at their default positions.
    enabled: bool,
    connected: bool,
    last_raw: Option<Quat>,
    last_update: Instant,
    /// With --verbose, when the state was last logged.
    last_log: Option<Instant>,
}

impl Tracker {
    fn new(source: Box<dyn OrientationSource>, filter: HeadFilter) -> Self {
        Tracker {
            source,
            filter,
            enabled: true,
            connected: false,
            last_raw: None,
            last_update: Instant::now(),
            last_log: None,
        }
    }

    /// The head orientation to render: the room-fixed default while not tracking.
    fn update(&mut self) -> Quat {
        let raw = self.source.poll();
        let connected = self.source.connected();
        if connected != self.connected {
            self.connected = connected;
            self.last_raw = None;
            self.filter.reset();
            self.filter.recenter();
        }
        // Keep feeding the last pose between tracker packets so smoothing settles.
        self.last_raw = raw.or(self.last_raw);
        let now = Instant::now();
        let dt = now.duration_since(std::mem::replace(&mut self.last_update, now)).as_secs_f64();
        match self.last_raw {
            Some(raw) if self.enabled && self.connected => self.filter.update(raw, dt),
            _ => Quat::IDENTITY,
        }
    }

    /// Once a second: raw and rendered heading, and the drift correction.
    fn log(&mut self, head: Quat) {
        if self.last_log.is_some_and(|t| t.elapsed() < Duration::from_secs(1)) {
            return;
        }
        self.last_log = Some(Instant::now());
        let Some(raw) = self.last_raw.filter(|_| self.connected) else { return };
        eprintln!(
            "raw heading {:+7.2}, rendered {:+6.2}, {}",
            raw.yaw_deg(),
            head.yaw_deg(),
            self.filter.describe()
        );
    }

    fn recenter(&mut self) {
        eprintln!("re-centring");
        self.filter.recenter();
    }

    fn toggle(&mut self) {
        self.enabled = !self.enabled;
        self.filter.reset();
        eprintln!("head tracking {}", if self.enabled { "on" } else { "off" });
    }

    fn status(&self, sink_present: bool) -> &'static str {
        match () {
            _ if !sink_present => "waiting for the spatial sink",
            _ if !self.enabled => "tracking off (SIGUSR2 to resume)",
            _ if !self.connected => "waiting for the head tracker",
            _ => "tracking",
        }
    }
}

fn make_source(args: &Args) -> Box<dyn OrientationSource> {
    match args.source {
        SourceKind::Fake => Box::new(FakeRotation::new(args.speed)),
        SourceKind::OpenTrack => match OpenTrackUdp::bind(args.listen) {
            Ok(source) => {
                eprintln!("listening for OpenTrack UDP on {}", args.listen);
                Box::new(source)
            }
            Err(e) => {
                eprintln!("cannot listen on {}: {e}", args.listen);
                std::process::exit(1);
            }
        },
        SourceKind::Maxwell => Box::new(MaxwellHid::new(args.hidraw.clone())),
    }
}

const SIGNALS: [i32; 4] = [libc::SIGINT, libc::SIGTERM, libc::SIGUSR1, libc::SIGUSR2];

fn signal_set() -> libc::sigset_t {
    unsafe {
        let mut set: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut set);
        for signal in SIGNALS {
            libc::sigaddset(&mut set, signal);
        }
        set
    }
}

/// PipeWire's loop reads signals through a signalfd, which only works if the
/// signals are blocked in every thread. Must run before any thread is spawned.
/// Between connections they stay blocked (pending) and `wait` takes them.
fn block_signals() {
    unsafe {
        libc::pthread_sigmask(libc::SIG_BLOCK, &signal_set(), std::ptr::null_mut());
    }
}

/// Two daemons would fight over the speakers: hold a lock while running.
fn lock_single_instance() -> File {
    let dir = std::env::var_os("XDG_RUNTIME_DIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    let path = dir.join("maxwell3d.lock");
    let file = OpenOptions::new().create(true).truncate(false).write(true).open(&path);
    let file = file.unwrap_or_else(|e| {
        eprintln!("cannot create {}: {e}", path.display());
        std::process::exit(1);
    });
    if unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } != 0 {
        eprintln!("maxwell3d is already running (it holds {})", path.display());
        std::process::exit(EXIT_ALREADY_RUNNING);
    }
    file
}

fn main() {
    let args = parse_args();
    let _lock = lock_single_instance();
    block_signals();
    pw::init();

    let filter = HeadFilter::new(args.smoothing_ms / 1000.0, args.auto_recenter_deg);
    let tracker = Rc::new(RefCell::new(Tracker::new(make_source(&args), filter)));
    let notifier = Rc::new(Notifier::from_env());
    notifier.send("READY=1");

    // Reconnect whenever PipeWire goes away (restarted, crashed, not up yet).
    loop {
        match run(&args, &tracker, &notifier) {
            Ok(Ended::Exit) => break,
            Ok(Ended::Disconnected) => {
                notifier.status("waiting for PipeWire");
            }
            Err(e) => {
                let status = format!("cannot connect to PipeWire ({e}), retrying");
                if notifier.status(&status) {
                    eprintln!("{status}");
                }
            }
        }
        if wait(RECONNECT_DELAY, &tracker, &notifier) {
            break;
        }
    }
    notifier.send("STOPPING=1");
}

/// Sleep while not connected to PipeWire, handling signals. Returns whether to exit.
fn wait(duration: Duration, tracker: &RefCell<Tracker>, notifier: &Notifier) -> bool {
    let deadline = Instant::now() + duration;
    loop {
        notifier.ping();
        // Keep reading the tracker so a short PipeWire restart doesn't look
        // like a tracker dropout (which re-centres).
        tracker.borrow_mut().update();
        let left = deadline.saturating_duration_since(Instant::now()).min(Duration::from_millis(250));
        if left.is_zero() {
            return false;
        }
        let timeout = libc::timespec { tv_sec: left.as_secs() as _, tv_nsec: left.subsec_nanos() as _ };
        match unsafe { libc::sigtimedwait(&signal_set(), std::ptr::null_mut(), &timeout) } {
            libc::SIGINT | libc::SIGTERM => return true,
            libc::SIGUSR1 => tracker.borrow_mut().recenter(),
            libc::SIGUSR2 => tracker.borrow_mut().toggle(),
            _ => {} // timed out or interrupted
        }
    }
}

enum Ended {
    /// SIGINT / SIGTERM.
    Exit,
    /// The connection to PipeWire was lost.
    Disconnected,
}

/// One connection to PipeWire: drive the sink's speakers until told to exit
/// or the connection is lost.
fn run(args: &Args, tracker: &Rc<RefCell<Tracker>>, notifier: &Rc<Notifier>) -> Result<Ended, pw::Error> {
    let mainloop = pw::main_loop::MainLoopRc::new(None)?;
    let context = pw::context::ContextRc::new(&mainloop, None)?;
    let core = context.connect_rc(None)?;
    let registry = core.get_registry_rc()?;

    // The filter-chain sink node, once it shows up in the registry.
    let node: Rc<RefCell<Option<(u32, pw::node::Node)>>> = Rc::new(RefCell::new(None));
    // Orientation the speakers were last set to; `None` sends the next one regardless.
    let last_sent: Rc<Cell<Option<Quat>>> = Rc::new(Cell::new(None));

    let _registry_listener = registry
        .add_listener_local()
        .global({
            let node = node.clone();
            let last_sent = last_sent.clone();
            let registry = registry.clone();
            let wanted = args.node_name.clone();
            move |global| {
                if global.type_ != ObjectType::Node
                    || global.props.and_then(|p| p.get("node.name")) != Some(wanted.as_str())
                {
                    return;
                }
                match registry.bind::<pw::node::Node, _>(global) {
                    Ok(proxy) => {
                        eprintln!("found {wanted} (id {})", global.id);
                        *node.borrow_mut() = Some((global.id, proxy));
                        // A new node starts from the positions in its config.
                        last_sent.set(None);
                    }
                    Err(e) => eprintln!("failed to bind {wanted}: {e}"),
                }
            }
        })
        .global_remove({
            let node = node.clone();
            move |id| {
                let mut node = node.borrow_mut();
                if node.as_ref().is_some_and(|(node_id, _)| *node_id == id) {
                    eprintln!("spatial sink went away, waiting for it to return");
                    *node = None;
                }
            }
        })
        .register();

    // Set on SIGINT/SIGTERM: stop tracking, the speakers are being reset.
    let exiting = Rc::new(Cell::new(false));
    let disconnected = Rc::new(Cell::new(false));

    let timer = mainloop.loop_().add_timer({
        let node = node.clone();
        let tracker = tracker.clone();
        let notifier = notifier.clone();
        let exiting = exiting.clone();
        let deadband = args.deadband_deg;
        let verbose = args.verbose;
        move |_| {
            notifier.ping();
            if exiting.get() {
                return;
            }
            let head = tracker.borrow_mut().update();
            if verbose {
                tracker.borrow_mut().log(head);
            }
            let node = node.borrow();
            notifier.status(tracker.borrow().status(node.is_some()));
            let Some((_, proxy)) = node.as_ref() else { return };
            // Every change makes the spatializers reload their filters without
            // a crossfade, which clicks: ignore tracker jitter below the deadband.
            if last_sent.get().is_some_and(|sent| sent.angle_to_deg(head) < deadband) {
                return;
            }
            set_speakers(proxy, head);
            last_sent.set(Some(head));
        }
    });
    let interval = Duration::from_secs_f64(1.0 / args.rate_hz);
    timer.update_timer(Some(interval), Some(interval)).into_result()?;

    let _recenter = mainloop.loop_().add_signal_local(Signal::USR1, {
        let tracker = tracker.clone();
        move || tracker.borrow_mut().recenter()
    });
    let _toggle = mainloop.loop_().add_signal_local(Signal::USR2, {
        let tracker = tracker.clone();
        move || tracker.borrow_mut().toggle()
    });

    // On exit put the speakers back where the config has them, then wait for
    // the server to confirm it processed the update before quitting.
    let exit_seq = Rc::new(Cell::new(None));
    let _core_listener = core
        .add_listener_local()
        .done({
            let mainloop = mainloop.clone();
            let exit_seq = exit_seq.clone();
            move |id, seq| {
                if id == pw::core::PW_ID_CORE && exit_seq.get() == Some(seq) {
                    mainloop.quit();
                }
            }
        })
        .error({
            let mainloop = mainloop.clone();
            let disconnected = disconnected.clone();
            move |id, _seq, res, message| {
                if id == pw::core::PW_ID_CORE && res == -libc::EPIPE {
                    eprintln!("lost the connection to PipeWire, reconnecting");
                    disconnected.set(true);
                    mainloop.quit();
                } else {
                    eprintln!("PipeWire error on object {id}: {message}");
                }
            }
        })
        .register();
    let request_exit = {
        let exiting = exiting.clone();
        let mainloop = mainloop.clone();
        let core = core.clone();
        move || {
            if exiting.replace(true) {
                // Second signal: don't wait for the server any more.
                mainloop.quit();
                return;
            }
            let Some((_, proxy)) = node.borrow_mut().take() else {
                mainloop.quit();
                return;
            };
            eprintln!("resetting speakers");
            set_speakers(&proxy, Quat::IDENTITY);
            match core.sync(0) {
                Ok(seq) => exit_seq.set(Some(seq)),
                Err(_) => mainloop.quit(),
            }
        }
    };
    let _sigint = mainloop.loop_().add_signal_local(Signal::INT, request_exit.clone());
    let _sigterm = mainloop.loop_().add_signal_local(Signal::TERM, request_exit);

    eprintln!("connected to PipeWire, waiting for node {}", args.node_name);
    mainloop.run();
    Ok(if disconnected.get() && !exiting.get() { Ended::Disconnected } else { Ended::Exit })
}

fn set_speakers(proxy: &pw::node::Node, head: Quat) {
    let bytes = speaker_props(head);
    proxy.set_param(ParamType::Props, 0, Pod::from_bytes(&bytes).expect("valid pod"));
}
