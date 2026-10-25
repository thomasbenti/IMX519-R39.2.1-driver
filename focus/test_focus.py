
import os
import subprocess
import time
from pathlib import Path

import numpy as np

WIDTH = 1920
HEIGHT = 1080
FRAME_SIZE = WIDTH * HEIGHT * 2

POSITIONS = [0, 256, 512, 768, 1024, 768, 512, 256, 0]
OUTPUT = Path("focus/tests")


def capture_frame(stream):
    # Read one complete RAW frame from the camera stream.
    data = stream.stdout.read(FRAME_SIZE)

    if len(data) != FRAME_SIZE:
        raise RuntimeError(
            "Camera stream ended unexpectedly. Check if /dev/video0 is busy."
        )

    return data


def sharpness(raw):
    # RG10 stores each 10-bit pixel in the lower bits of a 16-bit value.
    image = np.frombuffer(raw, dtype="<u2")
    image = image.reshape(HEIGHT, WIDTH) & 0x03FF

    # Use one green channel of the Bayer image to measure edge detail.
    green = image[0::2, 1::2].astype(np.float32)

    # Apply a simple discrete Laplacian.
    lap = (
        4 * green[1:-1, 1:-1]
        - green[:-2, 1:-1]
        - green[2:, 1:-1]
        - green[1:-1, :-2]
        - green[1:-1, 2:]
    )

    return float(np.var(lap))


OUTPUT.mkdir(parents=True, exist_ok=True)

# Keep the camera streaming so its POWER_EN signal remains high.
stream = subprocess.Popen(
    [
        "v4l2-ctl",
        "-d", "/dev/video0",
        "--stream-mmap",
        "--stream-to=-",
    ],
    stdout=subprocess.PIPE,
)

lens_fd = None

try:
    # Wait for the first camera frame before opening the actuator.
    capture_frame(stream)

    # Keep the actuator open throughout the entire focus sweep.
    lens_fd = os.open("/dev/v4l-subdev2", os.O_RDWR)

    for step, position in enumerate(POSITIONS):
        print(f"Testing focus position {position}", flush=True)

        subprocess.run(
            [
                "v4l2-ctl",
                "-d", "/dev/v4l-subdev2",
                f"--set-ctrl=focus_absolute={position}",
            ],
            check=True,
        )

        # Discard 15 frames so the lens can settle.
        for _ in range(15):
            capture_frame(stream)

        # Measure five frames at each focus position.
        frames = [capture_frame(stream) for _ in range(5)]
        scores = [sharpness(frame) for frame in frames]

        average = np.mean(scores)
        variation = np.std(scores)

        # Include the sweep step so repeated positions do not overwrite files.
        filename = OUTPUT / f"focus-{step:02d}-{position}.raw"
        filename.write_bytes(frames[0])

        print(
            f"Position {position}: "
            f"sharpness = {average:.2f}, "
            f"variation = {variation:.2f}",
            flush=True,
        )

finally:
    # Release the actuator while the camera is still powered.
    if lens_fd is not None:
        os.close(lens_fd)
        time.sleep(1)

    # Stop streaming only after allowing time for actuator suspension.
    stream.terminate()

    try:
        stream.wait(timeout=3)
    except subprocess.TimeoutExpired:
        stream.kill()
        stream.wait()

    stream.stdout.close()
