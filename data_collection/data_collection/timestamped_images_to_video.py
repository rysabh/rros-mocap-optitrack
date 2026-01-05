#!/usr/bin/env python3

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from pathlib import Path
import re
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2


@dataclass(frozen=True)
class TimestampedImage:
    path: Path
    timestamp_s: float


_FILENAME_RE = re.compile(
    r"^rgb_(?P<seq>\d+)_(?P<cam>\d+)_(?P<ts>\d+(?:\.\d+)?)\.png$",
    re.IGNORECASE,
)


def parse_timestamp_seconds(ts_token: str) -> float:
    """
    Parse timestamp token from `rgb_<seq>_<cam>_<timestamp>.png`.

    Supported:
    - float (contains "."): interpreted as seconds
    - int: interpreted by digit count (ns/us/ms/s)
    """
    raw = ts_token.strip()
    if "." in raw:
        return float(raw)

    value = int(raw)
    digits = len(raw)

    if digits >= 16:  # 1e15+ -> likely nanoseconds
        return value / 1e9
    if digits >= 14:  # microseconds
        return value / 1e6
    if digits >= 12:  # milliseconds
        return value / 1e3
    return float(value)  # seconds


def list_timestamped_images(rgb_dir: Path) -> List[TimestampedImage]:
    """
    List images in `rgb_dir` that match the expected name format and return them sorted by timestamp.
    """
    rgb_dir = Path(rgb_dir).expanduser()
    images: List[TimestampedImage] = []

    for p in rgb_dir.iterdir():
        if not p.is_file():
            continue
        m = _FILENAME_RE.match(p.name)
        if not m:
            continue
        ts_s = parse_timestamp_seconds(m.group("ts"))
        images.append(TimestampedImage(path=p, timestamp_s=ts_s))

    images.sort(key=lambda x: x.timestamp_s)
    return images


def default_output_path_for_rgb_dir(rgb_dir: Path) -> Path:
    """
    Default output naming (matches your traj/camera folder layout):
      <experiment_dir>/<camera_name>_rgb.mp4

    Example:
      .../traj10/camera_4/rgb  ->  .../traj10/camera_4_rgb.mp4

    If `rgb_dir` doesn't look like a ".../camera_X/rgb" folder, falls back to "<rgb_dir>/rgb.mp4".
    """
    rgb_dir = Path(rgb_dir).expanduser()
    if rgb_dir.name != "rgb":
        return rgb_dir / "rgb.mp4"

    camera_dir = rgb_dir.parent
    if not camera_dir.name.startswith("camera_"):
        return rgb_dir / "rgb.mp4"

    experiment_dir = camera_dir.parent
    return experiment_dir / f"{camera_dir.name}_rgb.mp4"


def _read_image(path: Path) -> "cv2.typing.MatLike":
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"Failed to read image: {path}")
    return img


def _frame_size(img) -> Tuple[int, int]:
    h, w = img.shape[:2]
    return (w, h)


def write_video_following_timeline(
    images: Sequence[TimestampedImage],
    output_path: Path,
    *,
    fps: float = 30.0,
    fourcc: str = "mp4v",
    overwrite: bool = False,
    resize_to_first: bool = True,
    verbose: bool = True,
) -> Path:
    """
    Create a constant-FPS video that follows the timeline implied by image timestamps.

    How it works:
    - Sort by timestamp (caller should already provide sorted sequence; we re-sort defensively).
    - Output has constant FPS (e.g. 30 or 15).
    - For each output time t = 0, 1/fps, 2/fps, ... we select the most recent image whose
      timestamp <= t and "hold" it until the next timestamp.
    """
    if fps <= 0:
        raise ValueError("fps must be > 0")

    images_sorted = sorted(images, key=lambda x: x.timestamp_s)
    if not images_sorted:
        raise ValueError("No images provided")

    output_path = Path(output_path).expanduser()
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"Output exists (set overwrite=True): {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    first_img = _read_image(images_sorted[0].path)
    size = _frame_size(first_img)

    t0 = images_sorted[0].timestamp_s
    times = [img.timestamp_s - t0 for img in images_sorted]
    duration_s = max(0.0, times[-1])
    out_frames = int(duration_s * fps) + 1

    writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*fourcc), float(fps), size)
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open VideoWriter: {output_path} (fourcc={fourcc})")

    last_loaded_idx = 0
    current_img = first_img
    try:
        for i in range(out_frames):
            t = i / float(fps)
            idx = max(0, bisect_right(times, t) - 1)
            if idx != last_loaded_idx:
                img = _read_image(images_sorted[idx].path)
                if resize_to_first and _frame_size(img) != size:
                    img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
                current_img = img
                last_loaded_idx = idx

            writer.write(current_img)
            if verbose and (i % int(max(1, fps)) == 0):
                pct = 100.0 * (i / max(1, out_frames - 1))
                print(f"{pct:5.1f}%  t={t:8.3f}s  {images_sorted[last_loaded_idx].path.name}", flush=True)
    finally:
        writer.release()

    if verbose:
        print(f"Wrote: {output_path}")
    return output_path


def render_video_from_rgb_dir(
    rgb_dir: Path,
    *,
    output_path: Optional[Path] = None,
    fps: float = 30.0,
    overwrite: bool = False,
    fourcc: str = "mp4v",
    resize_to_first: bool = True,
    verbose: bool = True,
) -> Path:
    """
    Convenience wrapper: scan `rgb_dir`, sort by timestamp, and write video.
    """
    rgb_dir = Path(rgb_dir).expanduser()
    if output_path is None:
        output_path = default_output_path_for_rgb_dir(rgb_dir)

    images = list_timestamped_images(rgb_dir)
    if not images:
        raise FileNotFoundError(f"No images found in {rgb_dir} matching rgb_<seq>_<cam>_<ts>.png")

    return write_video_following_timeline(
        images,
        output_path,
        fps=fps,
        fourcc=fourcc,
        overwrite=overwrite,
        resize_to_first=resize_to_first,
        verbose=verbose,
    )


if __name__ == "__main__":
    # Edit these paths as needed.
    rgb_dir_path = Path("/media/cam/SSK/temp/traj2/camera_4/rgb")
    
    out_path = default_output_path_for_rgb_dir(rgb_dir_path)

    print(f"Rendering video from \n{rgb_dir_path} \nto\n {out_path}")
    
    render_video_from_rgb_dir(
        rgb_dir_path,
        output_path=out_path,
        fps=30.0,
        overwrite=True,
        fourcc="mp4v",
        resize_to_first=True,
        verbose=True,
    )
