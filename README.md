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
  Higher hides tracker jitter but adds lag.
- `pkill -USR1 maxwell3d` makes the current heading the new "front" (bind it to a
  hotkey). Only the heading is reset; pitch and roll are kept.
- On Ctrl-C / SIGTERM the speakers are put back at their default positions before exiting.

Then play surround audio into the sink, for example:

    speaker-test -D pipewire -c 8 -t wav  # after choosing "Maxwell 3D" as the default output

If the speakers are ever left rotated (e.g. the daemon was killed with SIGKILL),
`systemctl --user restart pipewire` restores the positions from the config.

Angles use the SOFA convention: azimuth 0° = front, counter-clockwise (90° = left).
