#!/usr/bin/env python3

import argparse
import csv
import math
import shutil
from bisect import bisect_left
from pathlib import Path
from typing import List, Optional, Tuple


def _parse_float(value: str) -> Optional[float]:
    try:
        v = float(value)
    except Exception:
        return None
    if math.isnan(v) or math.isinf(v):
        return None
    return v


def _load_force_samples(csv_path: Path) -> Tuple[List[float], List[float], List[float], List[float]]:
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError("CSV has no header row")

        fieldnames = set(reader.fieldnames)
        if "time_s" in fieldnames:
            time_key = "time_s"
            t0 = None
        elif "sample_time_ns" in fieldnames:
            time_key = "sample_time_ns"
            t0 = None
        else:
            raise RuntimeError("CSV missing 'time_s' and 'sample_time_ns'")

        times: List[float] = []
        fx: List[float] = []
        fy: List[float] = []
        fz: List[float] = []

        for row in reader:
            raw_t = (row.get(time_key) or "").strip()
            if not raw_t:
                continue

            if time_key == "time_s":
                t = _parse_float(raw_t)
                if t is None:
                    continue
            else:
                ns = _parse_float(raw_t)
                if ns is None:
                    continue
                if t0 is None:
                    t0 = ns
                t = (ns - t0) / 1e9

            v_fx = _parse_float((row.get("fx") or "").strip())
            v_fy = _parse_float((row.get("fy") or "").strip())
            v_fz = _parse_float((row.get("fz") or "").strip())
            if v_fx is None or v_fy is None or v_fz is None:
                continue

            times.append(t)
            fx.append(v_fx)
            fy.append(v_fy)
            fz.append(v_fz)

    if not times:
        raise RuntimeError("No force samples found (missing fx/fy/fz or timestamps)")
    return times, fx, fy, fz


def _default_output_path(csv_path: Path) -> Path:
    take_dir = csv_path.parent
    take_name = take_dir.name
    return take_dir / f"{take_name}_forces.mp4"


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Create a moving-window force plot video from samples.csv.")
    parser.add_argument("--csv", type=Path, required=True, help="Path to samples.csv")
    parser.add_argument("--out", type=Path, default=None, help="Output mp4 path (default: <take>/<take>_forces.mp4)")
    parser.add_argument("--window-s", type=float, default=5.0, help="Visible history window in seconds")
    parser.add_argument("--fps", type=int, default=30, help="Output video FPS")
    parser.add_argument("--step", type=int, default=1, help="Use every Nth sample as a frame")
    parser.add_argument("--dpi", type=int, default=160, help="Video DPI")

    args = parser.parse_args(argv)
    csv_path: Path = args.csv.expanduser()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    out_path = (args.out.expanduser() if args.out else _default_output_path(csv_path)).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if not shutil.which("ffmpeg"):
        raise SystemExit("ffmpeg not found in PATH (required to write mp4 via matplotlib)")

    times, fx, fy, fz = _load_force_samples(csv_path)

    import matplotlib.pyplot as plt
    from matplotlib import animation

    window_s = max(0.1, float(args.window_s))
    fps = max(1, int(args.fps))
    step = max(1, int(args.step))

    fig, ax = plt.subplots(1, 1, figsize=(14, 6))
    ax.set_title("Forces [N]")
    ax.set_xlabel("Time [s] (relative)")
    ax.set_ylabel("Force [N]")
    ax.set_xlim(-window_s, 0.0)
    ax.grid(True)

    (l_fx,) = ax.plot([], [], label="Fx")
    (l_fy,) = ax.plot([], [], label="Fy")
    (l_fz,) = ax.plot([], [], label="Fz")
    ax.legend()

    frame_indices = list(range(0, len(times), step))

    def update(frame_i: int):
        idx = frame_indices[frame_i]
        t_now = times[idx]
        t_min = t_now - window_s
        start = bisect_left(times, t_min)
        end = idx + 1

        rel_t = [t - t_now for t in times[start:end]]
        fx_slice = fx[start:end]
        fy_slice = fy[start:end]
        fz_slice = fz[start:end]

        l_fx.set_data(rel_t, fx_slice)
        l_fy.set_data(rel_t, fy_slice)
        l_fz.set_data(rel_t, fz_slice)

        visible_vals = fx_slice + fy_slice + fz_slice
        if visible_vals:
            lo = min(visible_vals)
            hi = max(visible_vals)
            pad = 0.1 * max(1e-3, hi - lo)
            ax.set_ylim(lo - pad, hi + pad)

        return l_fx, l_fy, l_fz

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(frame_indices),
        interval=1000.0 / fps,
        blit=False,
        cache_frame_data=False,
    )

    writer = animation.FFMpegWriter(fps=fps, codec="libx264", bitrate=-1)
    ani.save(str(out_path), writer=writer, dpi=int(args.dpi))
    plt.close(fig)
    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

