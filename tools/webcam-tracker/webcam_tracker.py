#!/usr/bin/env python3
"""Webcam head tracker sending OpenTrack-compatible UDP packets.

Uses the MediaPipe face landmarker to estimate the head pose and sends it to
maxwell3d (`--source opentrack`) as OpenTrack's "UDP over network" output
does: six little-endian doubles x, y, z (cm), yaw, pitch, roll (degrees).

Angles follow maxwell3d's convention: positive yaw turns the head left,
positive pitch looks down, positive roll tilts the right ear down. They are
relative to the pose at startup; send SIGUSR1 to take the current pose as the
new reference. Frames are processed in memory only, nothing is stored.
"""

import argparse
import math
import signal
import socket
import struct
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions, vision

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/face_landmarker/"
    "face_landmarker/float16/1/face_landmarker.task"
)
MODEL_PATH = Path(__file__).with_name("face_landmarker.task")

# MediaPipe's metric space: x = image right (the user's left), y = up,
# z = towards the camera. maxwell3d's space: x = forward, y = left, z = up.
# The face model axes map the same way, so a rotation converts by conjugation.
CAMERA_TO_ROOM = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)


def load_model() -> Path:
    if not MODEL_PATH.exists():
        print(f"downloading {MODEL_URL}", file=sys.stderr)
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    return MODEL_PATH


def euler_deg(r: np.ndarray) -> tuple[float, float, float]:
    """Yaw, pitch, roll of a room-frame rotation matrix (intrinsic Z-Y-X)."""
    yaw = math.atan2(r[1, 0], r[0, 0])
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    roll = math.atan2(r[2, 1], r[2, 2])
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--camera", type=int, default=0, help="V4L2 device index (default 0)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4242)
    parser.add_argument("--verbose", action="store_true", help="print the pose once a second")
    args = parser.parse_args()

    landmarker = vision.FaceLandmarker.create_from_options(
        vision.FaceLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(load_model())),
            running_mode=vision.RunningMode.VIDEO,
            output_facial_transformation_matrixes=True,
        )
    )

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    # Uncompressed YUYV is often limited to ~15 fps at 640x480; MJPEG does 30.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        sys.exit(f"cannot open camera {args.camera}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)

    reference = None  # room-frame rotation that counts as "straight ahead"

    def recenter(*_):
        nonlocal reference
        reference = None
        print("re-centring", file=sys.stderr)

    signal.signal(signal.SIGUSR1, recenter)

    start = time.monotonic()
    frames, last_report = 0, start
    print(f"tracking, sending to {args.host}:{args.port}", file=sys.stderr)
    while True:
        ok, frame = cap.read()
        if not ok:
            sys.exit("camera read failed")
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        result = landmarker.detect_for_video(image, int((time.monotonic() - start) * 1000))
        frames += 1
        if not result.facial_transformation_matrixes:
            continue

        m = np.asarray(result.facial_transformation_matrixes[0])
        rotation = CAMERA_TO_ROOM @ m[:3, :3] @ CAMERA_TO_ROOM.T
        if reference is None:
            reference = rotation
        yaw, pitch, roll = euler_deg(reference.T @ rotation)
        x, y, z = m[:3, 3]
        sock.sendto(struct.pack("<6d", x, y, z, yaw, pitch, roll), target)

        now = time.monotonic()
        if args.verbose and now - last_report >= 1.0:
            fps = frames / (now - last_report)
            print(f"{fps:4.0f} fps  yaw {yaw:6.1f}  pitch {pitch:6.1f}  roll {roll:6.1f}", file=sys.stderr)
            frames, last_report = 0, now


if __name__ == "__main__":
    main()
