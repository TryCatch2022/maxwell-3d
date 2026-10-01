mod filter;
mod orientation;
mod source;

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::{Duration, Instant};

use pipewire as pw;
use pw::spa::param::ParamType;
use pw::spa::pod::{Object, Pod, Property, Value, serialize::PodSerializer};
use pw::loop_::Signal;
use pw::types::ObjectType;

use filter::HeadFilter;
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

enum SourceKind {
    Fake,
    OpenTrack,
    Maxwell,
}

struct Args {
    node_name: String,
    source: SourceKind,
    listen: std::net::SocketAddr,
    hidraw: Option<std::path::PathBuf>,
    rate_hz: f64,
    speed: f64,
    smoothing_ms: f64,
    deadband_deg: f64,
}

fn parse_args() -> Args {
    let mut args = Args {
        node_name: "effect_input.maxwell_3d".into(),
        source: SourceKind::Fake,
        listen: ([127, 0, 0, 1], 4242).into(),
        hidraw: None,
        rate_hz: 30.0,
        speed: 20.0,
        smoothing_ms: 30.0,
        deadband_deg: 1.0,
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut value = || it.next().unwrap_or_else(|| usage(&format!("{arg} needs a value")));
        match arg.as_str() {
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
            "--rate" => args.rate_hz = value().parse().unwrap_or_else(|_| usage("bad --rate")),
            "--speed" => args.speed = value().parse().unwrap_or_else(|_| usage("bad --speed")),
            "--deadband" => {
                args.deadband_deg = value().parse().unwrap_or_else(|_| usage("bad --deadband"))
            }
            "--smoothing" => {
                args.smoothing_ms = value().parse().unwrap_or_else(|_| usage("bad --smoothing"))
            }
            _ => usage(&format!("unknown argument {arg}")),
        }
    }
    args
}

fn usage(error: &str) -> ! {
    eprintln!("error: {error}");
    eprintln!(
        "usage: maxwell3d [--source fake|opentrack|maxwell] [--listen ADDR:PORT]\n\
         \x20                [--hidraw PATH] [--node NAME] [--rate HZ]\n\
         \x20                [--speed DEG_PER_SEC] [--smoothing MS] [--deadband DEG]"
    );
    eprintln!("send SIGUSR1 to make the current heading the new front");
    std::process::exit(2);
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

/// PipeWire's loop reads signals through a signalfd, which only works if the
/// signals are blocked in every thread. Must run before any thread is spawned.
fn block_signals() {
    unsafe {
        let mut set: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut set);
        for signal in [libc::SIGINT, libc::SIGTERM, libc::SIGUSR1] {
            libc::sigaddset(&mut set, signal);
        }
        libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());
    }
}

fn main() -> Result<(), pw::Error> {
    let args = parse_args();
    block_signals();
    pw::init();

    let mainloop = pw::main_loop::MainLoopRc::new(None)?;
    let context = pw::context::ContextRc::new(&mainloop, None)?;
    let core = context.connect_rc(None)?;
    let registry = core.get_registry_rc()?;

    // The filter-chain sink node, once it shows up in the registry.
    let node: Rc<RefCell<Option<(u32, pw::node::Node)>>> = Rc::new(RefCell::new(None));

    let _registry_listener = registry
        .add_listener_local()
        .global({
            let node = node.clone();
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

    let source: Box<dyn OrientationSource> = match args.source {
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
        SourceKind::Maxwell => {
            let Some(path) = args.hidraw.clone().or_else(MaxwellHid::find) else {
                eprintln!("no Audeze Maxwell headset or dongle found");
                std::process::exit(1);
            };
            match MaxwellHid::open(&path) {
                Ok(source) => {
                    eprintln!("head tracking from {}", path.display());
                    Box::new(source)
                }
                Err(e) => {
                    eprintln!("cannot open {}: {e}", path.display());
                    std::process::exit(1);
                }
            }
        }
    };
    let source = RefCell::new(source);
    let filter = Rc::new(RefCell::new(HeadFilter::new(args.smoothing_ms / 1000.0)));
    // Set on SIGINT/SIGTERM: stop tracking, the speakers are being reset.
    let exiting = Rc::new(Cell::new(false));

    let timer = mainloop.loop_().add_timer({
        let node = node.clone();
        let filter = filter.clone();
        let exiting = exiting.clone();
        let last = Cell::new(Instant::now());
        // Keep feeding the last pose between tracker packets so smoothing settles.
        let last_raw = Cell::new(None);
        let last_sent = Cell::new(None);
        let deadband = args.deadband_deg;
        move |_| {
            if exiting.get() {
                return;
            }
            let Some(raw) = source.borrow_mut().poll().or(last_raw.get()) else { return };
            last_raw.set(Some(raw));
            let now = Instant::now();
            let dt = now.duration_since(last.replace(now)).as_secs_f64();
            let head = filter.borrow_mut().update(raw, dt);
            // Every change makes the spatializers reload their filters without
            // a crossfade, which clicks: ignore tracker jitter below the deadband.
            if last_sent.get().is_some_and(|sent: Quat| sent.angle_to_deg(head) < deadband) {
                return;
            }
            if let Some((_, proxy)) = node.borrow().as_ref() {
                set_speakers(proxy, head);
                last_sent.set(Some(head));
            }
        }
    });
    let interval = Duration::from_secs_f64(1.0 / args.rate_hz);
    timer.update_timer(Some(interval), Some(interval)).into_result()?;

    let _recenter = mainloop.loop_().add_signal_local(Signal::USR1, move || {
        eprintln!("re-centring");
        filter.borrow_mut().recenter();
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
        .register();
    let request_exit = {
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

    eprintln!("waiting for node {}", args.node_name);
    mainloop.run();
    Ok(())
}

fn set_speakers(proxy: &pw::node::Node, head: Quat) {
    let bytes = speaker_props(head);
    proxy.set_param(ParamType::Props, 0, Pod::from_bytes(&bytes).expect("valid pod"));
}
