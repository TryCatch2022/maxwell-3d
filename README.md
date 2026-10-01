# maxwell-3d

Head-tracked spatial audio on Linux for the Audeze Maxwell 2.

- `pipewire/maxwell-3d-sink.conf`: a 7.1 virtual sink ("Maxwell 3D (7.1 spatial)").
  Each channel is rendered binaurally by a PipeWire `sofa` spatializer and the
  result is mixed down to stereo for the headset.
- `maxwell3d`: a daemon that reads the head orientation and rotates the seven
  virtual speakers around the listener by rewriting the spatializers' `Azimuth` /
  `Elevation` controls. For now it only has a fake source that turns the head slowly.

## Try it

The sink uses the SADIE II KU100 HRTF (University of York, Apache-2.0):

    mkdir -p ~/.local/share/pipewire/hrtf-sofa
    curl -L -o /tmp/D1_HRIR_SOFA.zip 'https://zenodo.org/records/12092466/files/D1_HRIR_SOFA.zip?download=1'
    unzip -j /tmp/D1_HRIR_SOFA.zip D1_HRIR_SOFA/D1_48K_24bit_256tap_FIR_SOFA.sofa -d ~/.local/share/pipewire/hrtf-sofa/

The config refers to it by absolute path (the sofa plugin doesn't expand `~`),
so fix the `filename` entries if your home isn't `/home/yngwe`. If the file
can't be opened, the sink still shows up but plays silence.

    mkdir -p ~/.config/pipewire/pipewire.conf.d
    ln -s $PWD/pipewire/maxwell-3d-sink.conf ~/.config/pipewire/pipewire.conf.d/
    systemctl --user restart pipewire

    cargo run --release -- --speed 20     # degrees per second, --rate 30 Hz by default

- `--smoothing MS` (default 30): time constant of the orientation smoothing; 0 disables it.
- `--deadband DEG` (default 1): only move the speakers once the head has turned this
  far since the last update. Each update makes the spatializers swap filters
  without a crossfade, which is audible as clicks, so tracker jitter must not reach them.
  Higher hides tracker jitter but adds lag.
- `pkill -USR1 maxwell3d` makes the current heading the new "front" (bind it to a
  hotkey). Only the heading is reset; pitch and roll are kept.
- On Ctrl-C / SIGTERM the speakers are put back at their default positions before exiting.

### Head tracking with a webcam

`tools/webcam-tracker` estimates the head pose with MediaPipe and sends it as
OpenTrack UDP packets (so a real OpenTrack install can be used instead):

    python3 -m venv tools/webcam-tracker/.venv
    tools/webcam-tracker/.venv/bin/pip install -r tools/webcam-tracker/requirements.txt
    tools/webcam-tracker/.venv/bin/python tools/webcam-tracker/webcam_tracker.py --verbose

    cargo run --release -- --source opentrack   # listens on 127.0.0.1:4242

The face landmarker model is downloaded next to the script on first run. Look
straight ahead when starting the tracker: the first pose is "front". `pkill -USR1 -f
webcam_tracker.py` re-centres all three axes (useful when the camera is above
the screen), `pkill -USR1 maxwell3d` re-centres the heading only.

### Head tracking from the Maxwell

Works with the headset plugged in over USB-C (in USB audio mode) or
wirelessly through the dongle:

    cargo run --release -- --source maxwell

This switches the headset's head tracking stream on (the same register write
the Windows apps make) and switches it off again on exit. Your pose when it
starts is "front". When both are plugged in the dongle is used, unless the
headset on USB-C is in USB audio mode (it isn't while the dongle is connected); `--hidraw /dev/hidrawN` picks a node by hand. Reading the
hidraw nodes needs `udev/70-audeze-maxwell.rules` in `/etc/udev/rules.d/`.

Then play surround audio into the sink, for example:

    speaker-test -D pipewire -c 8 -t wav  # after choosing "Maxwell 3D" as the default output

If the speakers are ever left rotated (e.g. the daemon was killed with SIGKILL),
`systemctl --user restart pipewire` restores the positions from the config.

Angles use the SOFA convention: azimuth 0° = front, counter-clockwise (90° = left).
