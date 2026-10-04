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

The room is darker than the direct sound (absorption) and has its modes in the
bass, which colours the result once there is a fair amount of it. Unless
--no-eq is given, one filter (the same for every speaker and ear, so spatial
cues are kept) makes the room part's average response flat, which is the
tonal balance of the diffuse-field equalised SADIE HRIRs: 1/3-octave smoothed, within +-10 dB, between 60 Hz and 16 kHz, keeping the
room's level.

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

# Tonal correction of the room part.
EQ_LIMIT_DB = 10.0
EQ_LOW, EQ_HIGH = 60.0, 16000.0  # full correction in between, fading out to 0 dB an octave beyond
EQ_TAPS = 4096


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


def smooth(freqs, power, fraction=3):
    """Mean of `power` over a 1/`fraction` octave around each frequency."""
    cum = np.concatenate([[0.0], np.cumsum(power)])
    half = 2 ** (1 / (2 * fraction))
    lo = np.searchsorted(freqs, freqs / half)
    hi = np.maximum(np.searchsorted(freqs, freqs * half, side="right"), lo + 1)
    return (cum[hi] - cum[lo]) / (hi - lo)


def average_power(responses, n):
    """Power spectrum averaged over speakers and ears."""
    return np.mean([np.abs(np.fft.rfft(r, n, axis=-1)) ** 2 for r in responses], axis=(0, 1))


def tone_correction(rooms, rate):
    """Minimum-phase FIR making the rooms' average response flat."""
    n = max(r.shape[1] for r in rooms)
    freqs = np.fft.rfftfreq(n, 1 / rate)
    p_room = average_power(rooms, n)
    corr = -10 * np.log10(smooth(freqs, p_room))
    corr -= np.mean(corr[(freqs > 500) & (freqs < 1000)])  # limit relative to the mids
    # Fade the correction out below EQ_LOW and above EQ_HIGH (over an octave).
    octaves = np.log2(np.maximum(freqs, 1) / EQ_LOW).clip(-1, 0) + np.log2(EQ_HIGH / np.maximum(freqs, 1)).clip(-1, 0)
    corr = np.clip(corr, -EQ_LIMIT_DB, EQ_LIMIT_DB) * (1 + octaves).clip(0, 1)
    # Keep the room's level (200 Hz - 8 kHz), so the direct/room balance holds.
    band = (freqs > 200) & (freqs < 8000)
    corr -= 10 * np.log10(np.sum(p_room[band] * 10 ** (corr[band] / 10)) / np.sum(p_room[band]))
    # Minimum phase from the magnitude via the real cepstrum.
    log_mag = np.concatenate([corr, corr[-2:0:-1]]) * np.log(10) / 20
    cep = np.fft.ifft(log_mag).real
    fold = np.zeros_like(cep)
    fold[0], fold[n // 2] = cep[0], cep[n // 2]
    fold[1:n // 2] = 2 * cep[1:n // 2]
    fir = np.fft.ifft(np.exp(np.fft.fft(fold))).real[:EQ_TAPS]
    fir[-EQ_TAPS // 8:] *= np.hanning(EQ_TAPS // 4)[EQ_TAPS // 8:]
    return fir, freqs, corr


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
    ap.add_argument("--no-eq", action="store_true", help="leave the room part's tonal balance as measured")
    args = ap.parse_args()

    hrir, hpos, hrate = load(args.hrir)
    brir, bpos, brate = load(args.brir)
    if hrate != brate:
        raise SystemExit(f"sample rates differ: {hrate} vs {brate}")
    rate = brate
    ms = lambda t: int(round(t * rate / 1000))
    os.makedirs(args.out, exist_ok=True)

    end_fade = np.ones(brir.shape[-1])
    end_fade[-ms(END_FADE):] = np.linspace(1, 0, ms(END_FADE))
    directs, rooms = [], []
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
        room = b * gate * end_fade

        # Same direct-to-room ratio as measured, at the HRIR's level and timing.
        scale = np.sqrt(window_energy(h, h_peak, rate) / window_energy(b, b_peak, rate))
        shift = h_peak - b_peak
        out = np.zeros_like(room)
        if shift >= 0:
            out[:, shift:] = room[:, : room.shape[1] - shift]
        else:
            out[:, :shift] = room[:, -shift:]
        directs.append(h)
        rooms.append(out * scale)

    if not args.no_eq:
        fir, freqs, corr = tone_correction(rooms, rate)
        n = rooms[0].shape[1]
        size = n + len(fir)
        rooms = [np.fft.irfft(np.fft.rfft(r, size, axis=-1) * np.fft.rfft(fir, size), size, axis=-1)[:, :n] * end_fade
                 for r in rooms]
        at = lambda f: corr[np.argmin(np.abs(freqs - f))]
        print("room tonal correction: " + ", ".join(
            f"{f if f < 1000 else f // 1000}{'' if f < 1000 else 'k'} {at(f):+.1f}"
            for f in (60, 100, 150, 200, 300, 500, 1000, 2000, 4000, 8000, 12000, 16000)) + " dB")

    for (name, azimuth), h, out in zip(SPEAKERS, directs, rooms):
        path = os.path.join(args.out, f"room_{name}.wav")
        write_float_wav(path, out, rate)
        direct = window_energy(h, onset(h), rate)
        ratio = 10 * np.log10(np.sum(out ** 2) / direct)
        print(f"{name}: BRIR at {bpos[nearest(bpos, azimuth)][0]:.0f} deg, room {ratio:+.1f} dB vs direct -> {path}")


if __name__ == "__main__":
    main()
