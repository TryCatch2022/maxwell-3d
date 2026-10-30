# maxwell-3d

Head-tracked spatial audio on Linux for the Audeze Maxwell 2.

- `pipewire/maxwell-3d-sink.conf`: a 7.1 virtual sink ("Maxwell 3D (7.1 spatial)").
  Each channel is rendered binaurally by a PipeWire `sofa` spatializer and the
  result is mixed down to stereo for the headset.
- `maxwell3d`: a daemon that reads the head orientation (from the headset, a
  webcam / OpenTrack, or a fake source for testing) and rotates the seven
  virtual speakers around the listener by rewriting the spatializers'
  `Azimuth` / `Elevation` controls.

## Install

Needs Rust (cargo), the PipeWire development files (`libpipewire-0.3-dev` /
`pipewire-devel`, plus clang for the bindings), curl and unzip.

    ./install.sh

This builds the daemon and installs it for your user, as described at the top
of [install.sh](install.sh): the binary and `maxwell3d-hrtf` in `~/.local/bin`, the
sink config, the udev rule that gives you access to the headset (asks for sudo
once), and a systemd user service that starts at login. It also downloads the
default HRTF (SADIE II KU100, University of York, Apache-2.0) if it's missing,
and restarts PipeWire when the sink config changed. Run it again after pulling
changes; your HRTF choice is kept. `./install.sh --uninstall` removes it all
except the downloaded HRTFs.

Then choose "Maxwell 3D (7.1 spatial)" as the output and play surround audio:

    speaker-test -D pipewire -c 8 -t wav

### Using it

    systemctl --user status maxwell3d          # what it's doing ("tracking", "waiting for the head tracker", ...)
    journalctl --user -u maxwell3d -f          # its log
    systemctl --user kill -s USR1 maxwell3d    # re-centre: the current pose becomes "front, level"
    systemctl --user kill -s USR2 maxwell3d    # head tracking off / on

The two signal commands are worth binding to hotkeys (`pkill -USR1 -x maxwell3d`
does the same). Re-centre while looking straight ahead with your head level: the
whole pose is reset, since the headset's notion of "level" is just however it
sat when tracking started.
Small heading drift is corrected automatically (`--auto-recenter`, below).
With tracking off the speakers go back to their fixed positions around your head.

The daemon is meant to be left running:

- It waits for the headset if none is connected, and copes with it coming and
  going: unplugging the dongle or cable, the headset switching off or going out
  of range, or moving between the dongle and USB-C. While no tracking data comes
  in the speakers stay at their fixed positions; when it resumes, your heading
  at that moment becomes "front".
- It reconnects when PipeWire restarts (e.g. `maxwell3d-hrtf`), and waits for the
  sink if it isn't loaded.
- On SIGINT / SIGTERM it puts the speakers back at their default positions.
  If it is killed outright, systemd restarts it and the positions are fixed then
  (or `systemctl --user restart pipewire` resets them).
- Only one instance runs at a time; stop the service before running it by hand.
- systemd restarts it if it crashes or its main loop stops responding (watchdog).

### Options

Change them with `systemctl --user edit maxwell3d` (an empty `ExecStart=` line,
then `ExecStart=%h/.local/bin/maxwell3d --source maxwell --deadband 2`), or pass
them when running it by hand (`cargo run --release -- ...`):

- `--source maxwell|opentrack|fake` (default maxwell): where the head pose comes
  from, see below. `fake` turns the head at `--speed` degrees per second.
- `--smoothing MS` (default 30): time constant of the orientation smoothing; 0 disables it.
- `--deadband DEG` (default 1): only move the speakers once the head has turned this
  far since the last update. Each update makes the spatializers swap filters
  without a crossfade, which is audible as clicks, so tracker jitter must not reach them.
  Higher hides tracker jitter but adds lag.
- `--auto-recenter DEG` (default 15, 0 disables): counter the headset's heading
  drift. Worn, its heading slides by itself (up to ~0.2°/s, i.e. front ends up
  to one side within a minute) and it loses about 0.5-1% of every turn, either
  way. The slide is measured whenever you hold your head still for a few
  seconds and cancelled from then on. On top of that, where your head rests
  within this many degrees of front is learnt slowly and becomes the new
  front, so front settles on where you usually look while short glances
  hardly count. That correction is applied only while you turn your head, so
  the shift isn't noticeable and causes no clicks while you sit still.
  Looking somewhere further off for a long time doesn't move front.
- `--rate HZ` (default 30): how often the speakers are updated.
- `--verbose`: log the head tracking state once a second (raw and rendered
  heading, drift correction).

### Trying other HRTFs

The KU100 dummy head (D1) is a generic choice; a human subject whose ears are
closer to yours can localise better (front stays in front, rear isn't above).
SADIE II has 18 human subjects (H3-H20) in the same record, e.g.:

    for h in H3 H4 H5 H6 H7; do
      curl -L -o /tmp/$h.zip "https://zenodo.org/records/12092466/files/${h}_HRIR_SOFA.zip?download=1"
      unzip -j /tmp/$h.zip "${h}_HRIR_SOFA/${h}_48K_24bit_256tap_FIR_SOFA.sofa" -d ~/.local/share/pipewire/hrtf-sofa/
    done

    maxwell3d-hrtf        # list, current one marked
    maxwell3d-hrtf H5     # switch (restarts PipeWire)

Known subjects get a gain matched to D1's loudness so the louder one doesn't
win the comparison by default.

### Room sound

The HRTF spatializers are anechoic. Reflections and reverb from a real room
(SADIE II D1's BRIRs, the same KU100 head) are added through fixed convolvers,
which don't follow the head and so never click. The room alone is dark and
boomy, so the script equalises it flat (`--no-eq` to skip). Generate the
impulse responses once:

    curl -L -o /tmp/D1_BRIR.zip 'https://zenodo.org/records/12092466/files/D1_BRIR_SOFA.zip?download=1'
    mkdir -p ~/.local/share/pipewire/hrtf-sofa/room
    unzip -j /tmp/D1_BRIR.zip D1_BRIR_SOFA/D1_48K_24bit_0.3s_FIR_SOFA.sofa -d ~/.local/share/pipewire/hrtf-sofa/room/
    python3 -m venv tools/room-tails/.venv
    tools/room-tails/.venv/bin/pip install -r tools/room-tails/requirements.txt
    tools/room-tails/.venv/bin/python tools/room-tails/make_room_tails.py \
        ~/.local/share/pipewire/hrtf-sofa/D1_48K_24bit_256tap_FIR_SOFA.sofa \
        ~/.local/share/pipewire/hrtf-sofa/room/D1_48K_24bit_0.3s_FIR_SOFA.sofa \
        ~/.local/share/pipewire/hrtf-sofa/room
    systemctl --user restart pipewire   # load them

The `outL`/`outR` mixers set the balance: `Gain 1` is the direct sound,
`Gain 2` the room. The default is 0.5 / 2.0, four times the measured room
(which is fairly dry) at the same overall level as without room; full-scale
stereo then peaks at -9 dBFS in the worst case, so there is room to go up.
Can be changed while playing:

    pw-cli set-param effect_input.maxwell_3d Props '{ params = [ "outL:Gain 2" 0.0 "outR:Gain 2" 0.0 ] }'   # dry

### Head tracking with a webcam

`tools/webcam-tracker` estimates the head pose with MediaPipe and sends it as
OpenTrack UDP packets (so a real OpenTrack install can be used instead):

    python3 -m venv tools/webcam-tracker/.venv
    tools/webcam-tracker/.venv/bin/pip install -r tools/webcam-tracker/requirements.txt
    tools/webcam-tracker/.venv/bin/python tools/webcam-tracker/webcam_tracker.py --verbose

    maxwell3d --source opentrack   # listens on 127.0.0.1:4242 (stop the service first)

The face landmarker model is downloaded next to the script on first run. Look
straight ahead when starting the tracker: the first pose is "front". `pkill -USR1 -f
webcam_tracker.py` re-centres all three axes (useful when the camera is above
the screen), `pkill -USR1 maxwell3d` does the same in maxwell3d.

### Head tracking from the Maxwell

The default source. Works with the headset plugged in over USB-C (in USB audio
mode) or wirelessly through the dongle. The daemon switches the headset's head
tracking stream on (the same register write the Windows apps make) and switches
it off again on exit. When both are plugged in the dongle is used, unless the
headset on USB-C is in USB audio mode (it isn't while the dongle is connected);
`--hidraw /dev/hidrawN` picks a node by hand. Reading the hidraw nodes needs
`udev/70-audeze-maxwell.rules` in `/etc/udev/rules.d/` (install.sh puts it there).

Angles use the SOFA convention: azimuth 0° = front, counter-clockwise (90° = left).
