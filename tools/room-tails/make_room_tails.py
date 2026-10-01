#!/usr/bin/env python3
"""Turn SADIE II room measurements (BRIRs) into "room only" impulse responses.

The Maxwell 3D sink renders the direct sound of each virtual speaker with a
head-tracked HRTF spatializer. This script provides the room around it: for
each speaker it takes the BRIR measured nearest to the speaker's direction,
removes the direct sound (keeping reflections and reverb), lines it up with
the direct sound of the anechoic HRIR and scales it so that direct and room
keep the measured ratio. The sink plays these through fixed convolvers, which
never reload, so head tracking doesn't make them click. The room doesn't turn
with the head; reverb arrives from all around anyway.

Both files must come from the same head (D1, the KU100, has both):

    python3 -m venv tools/room-tails/.venv
    tools/room-tails/.venv/bin/pip install -r tools/room-tails/requirements.txt
    tools/room-tails/.venv/bin/python tools/room-tails/make_room_tails.py \\
        ~/.local/share/pipewire/hrtf-sofa/D1_48K_24bit_256tap_FIR_SOFA.sofa \\
        ~/.local/share/pipewire/hrtf-sofa/room/D1_48K_24bit_0.3s_FIR_SOFA.sofa \\
        ~/.local/share/pipewire/hrtf-sofa/room

Writes room_<speaker>.wav (stereo, 32-bit float) per speaker. They are scaled
like the HRIRs, so the convolvers take the same gain as the spatializers.
"""
import argparse
import os
import struct

import h5py
import numpy as np

# Speaker name and azimuth (SOFA convention: counter-clockwise, 90 = left).
SPEAKERS = [("FL", 30), ("FR", 330), ("FC", 0), ("SL", 90), ("SR", 270), ("RL", 150), ("RR", 210)]

# Around the direct sound's peak: what counts as direct (for the level) and
# where the room part starts, in ms.
DIRECT_BEFORE, DIRECT_AFTER = 1.0, 2.5
CUT_START, CUT_FADE = 1.5, 1.0
END_FADE = 10.0


def load(path):
    with h5py.File(path, "r") as f:
        ir = f["Data.IR"][:].astype(np.float64)
        pos = f["SourcePosition"][:]
        rate = float(np.ravel(f["Data.SamplingRate"][:])[0])
    return ir, pos, rate


def nearest(pos, azimuth):
    """Measurement nearest to (azimuth, elevation 0)."""
    distance = np.abs((pos[:, 0] - azimuth + 180) % 360 - 180) + np.abs(pos[:, 1])
    return int(np.argmin(distance))


def onset(ir):
    """Index of the direct sound's peak in a 2 x N response."""
    return int(np.argmax(np.max(np.abs(ir), axis=0)))


def window_energy(ir, peak, rate):
    a = max(0, peak - int(DIRECT_BEFORE * rate / 1000))
    b = peak + int(DIRECT_AFTER * rate / 1000)
    return float(np.sum(ir[:, a:b] ** 2))


def write_float_wav(path, data, rate):
    """Stereo 32-bit float WAV (format tag 3); `data` is 2 x N."""
    frames = data.T.astype("<f4").tobytes()
    fmt = struct.pack("<HHIIHH", 3, 2, int(rate), int(rate) * 8, 8, 32)
    fact = struct.pack("<I", data.shape[1])
    body = b"WAVE" + b"fmt " + struct.pack("<I", len(fmt)) + fmt
    body += b"fact" + struct.pack("<I", len(fact)) + fact
    body += b"data" + struct.pack("<I", len(frames)) + frames
    with open(path, "wb") as f:
        f.write(b"RIFF" + struct.pack("<I", len(body)) + body)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("hrir", help="anechoic HRIR SOFA file used by the spatializers")
    ap.add_argument("brir", help="BRIR SOFA file of the same head")
    ap.add_argument("out", help="output directory")
    args = ap.parse_args()

    hrir, hpos, hrate = load(args.hrir)
    brir, bpos, brate = load(args.brir)
    if hrate != brate:
        raise SystemExit(f"sample rates differ: {hrate} vs {brate}")
    rate = brate
    ms = lambda t: int(round(t * rate / 1000))
    os.makedirs(args.out, exist_ok=True)

    for name, azimuth in SPEAKERS:
        b = brir[nearest(bpos, azimuth)]
        h = hrir[nearest(hpos, azimuth)]
        b_peak, h_peak = onset(b), onset(h)

        # Room part: silence through the direct sound, then a raised-cosine fade in.
        gate = np.zeros(b.shape[1])
        start = b_peak + ms(CUT_START)
        fade = ms(CUT_FADE)
        gate[start:start + fade] = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, fade))
        gate[start + fade:] = 1.0
        gate[-ms(END_FADE):] *= np.linspace(1, 0, ms(END_FADE))
        room = b * gate

        # Same direct-to-room ratio as measured, at the HRIR's level and timing.
        scale = np.sqrt(window_energy(h, h_peak, rate) / window_energy(b, b_peak, rate))
        shift = h_peak - b_peak
        out = np.zeros_like(room)
        if shift >= 0:
            out[:, shift:] = room[:, : room.shape[1] - shift]
        else:
            out[:, :shift] = room[:, -shift:]
        out *= scale

        path = os.path.join(args.out, f"room_{name}.wav")
        write_float_wav(path, out, rate)
        direct = window_energy(h, h_peak, rate)
        ratio = 10 * np.log10(np.sum(out ** 2) / direct)
        print(f"{name}: BRIR at {bpos[nearest(bpos, azimuth)][0]:.0f} deg, room {ratio:+.1f} dB vs direct -> {path}")


if __name__ == "__main__":
    main()
