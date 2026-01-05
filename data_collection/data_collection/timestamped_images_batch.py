#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple, Optional

"""
Batch helpers for turning timestamped RGB image folders into videos.

Expected folder layout (examples):
  <root>/traj10/camera_4/rgb/rgb_000001_4_1700000000.123.png
  <root>/traj10/camera_4/rgb/rgb_000002_4_1700000000.156.png

Output naming is delegated to `default_output_path_for_rgb_dir()`:
  .../traj10/camera_4/rgb  ->  .../traj10/camera_4_rgb.mp4

This file is intentionally NOT a ROS2 node. Just run it with python3 or import the functions.
"""

from timestamped_images_to_video import default_output_path_for_rgb_dir, render_video_from_rgb_dir


@dataclass(frozen=True)
class RenderJob:
    rgb_dir: Path
    output_path: Path


def find_rgb_dirs(root: Path) -> List[Path]:
    """
    Returns every directory named exactly 'rgb' under root (recursive).
    """
    root = Path(root).expanduser()
    return sorted([p for p in root.rglob("rgb") if p.is_dir()])


def make_jobs_for_all_rgb_dirs(root: Path) -> List[RenderJob]:
    jobs: List[RenderJob] = []
    for rgb_dir in find_rgb_dirs(root):
        jobs.append(RenderJob(rgb_dir=rgb_dir, output_path=default_output_path_for_rgb_dir(rgb_dir)))
    return jobs


def make_jobs_for_trajectories_and_cameras(
    root: Path,
    *,
    trajectories: Sequence[str],
    cameras: Sequence[str],
) -> List[RenderJob]:
    """
    Creates jobs for:
      <root>/<traj>/<camera>/rgb

    Examples:
      trajectories=["traj10", "traj11"]
      cameras=["camera_1", "camera_4"]
    """
    root = Path(root).expanduser()
    jobs: List[RenderJob] = []
    for traj_name in trajectories:
        for cam_name in cameras:
            rgb_dir = root / str(traj_name) / str(cam_name) / "rgb"
            if not rgb_dir.is_dir():
                continue
            jobs.append(RenderJob(rgb_dir=rgb_dir, output_path=default_output_path_for_rgb_dir(rgb_dir)))
    return jobs


def _run_one_job(
    job: RenderJob,
    *,
    fps: float,
    overwrite: bool,
    fourcc: str,
    resize_to_first: bool,
    verbose: bool,
) -> Tuple[str, str, Optional[str]]:
    """
    Helper so it can be used both sequentially and with ProcessPoolExecutor.
    Returns: (rgb_dir, output_path, error_or_none)
    """
    try:
        render_video_from_rgb_dir(
            job.rgb_dir,
            output_path=job.output_path,
            fps=fps,
            overwrite=overwrite,
            fourcc=fourcc,
            resize_to_first=resize_to_first,
            verbose=verbose,
        )
        return (str(job.rgb_dir), str(job.output_path), None)
    except Exception as exc:
        return (str(job.rgb_dir), str(job.output_path), str(exc))


def run_jobs(
    jobs: Sequence[RenderJob],
    *,
    fps: float = 30.0,
    overwrite: bool = False,
    fourcc: str = "mp4v",
    resize_to_first: bool = True,
    verbose: bool = True,
    parallel: bool = False,
    max_workers: int = 4,
) -> None:
    """
    Runs jobs sequentially by default.

    If `parallel=True`, runs multiple jobs concurrently (1 process per job).
    This parallelizes across folders/cameras/trajectories, not within a single video.
    """
    if not parallel:
        for job in jobs:
            if verbose:
                print(f"\n=== {job.rgb_dir} -> {job.output_path} ===")
            rgb_dir, _out_path, err = _run_one_job(
                job,
                fps=fps,
                overwrite=overwrite,
                fourcc=fourcc,
                resize_to_first=resize_to_first,
                verbose=verbose,
            )
            if err:
                print(f"[ERROR] {rgb_dir}: {err}")
        return

    from concurrent.futures import ProcessPoolExecutor, as_completed

    workers = max(1, int(max_workers))
    if verbose:
        print(f"Running {len(jobs)} jobs with {workers} workers...")

    with ProcessPoolExecutor(max_workers=workers) as ex:
        futs = [
            ex.submit(
                _run_one_job,
                job,
                fps=fps,
                overwrite=overwrite,
                fourcc=fourcc,
                resize_to_first=resize_to_first,
                verbose=False,
            )
            for job in jobs
        ]
        for fut in as_completed(futs):
            rgb_dir, out_path, err = fut.result()
            if err:
                print(f"[ERROR] {rgb_dir}: {err}")
            elif verbose:
                print(f"[OK] {rgb_dir} -> {out_path}")


if __name__ == "__main__":
    # ----------------------------
    # Examples (edit + run)
    # ----------------------------
    #
    # Run:
    #   python3 timestamped_images_batch.py
    #
    # Tip: start by setting `root_dir` to your dataset root.

    root_dir = Path("/media/cam/SSK/temp")

    # Example 1) One experiment folder (traj10), specific cameras
    # Produces:
    #   <root>/traj10/camera_1_rgb.mp4
    #   <root>/traj10/camera_2_rgb.mp4
    #   <root>/traj10/camera_3_rgb.mp4
    #   <root>/traj10/camera_4_rgb.mp4
    # trajectories = ["traj10"]
    # cameras = ["camera_1", "camera_2", "camera_3", "camera_4"]
    # jobs = make_jobs_for_trajectories_and_cameras(root_dir, trajectories=trajectories, cameras=cameras)
    # run_jobs(jobs, fps=30.0, overwrite=True, fourcc="mp4v", resize_to_first=True, verbose=True)

    # Example 2) Traj list and camera list (easy to change ranges)
    trajectories = [f"traj{i}" for i in range(10, 56)]  # traj1..traj10
    cameras = ["camera_1", "camera_2", "camera_3", "camera_4"]
    jobs = make_jobs_for_trajectories_and_cameras(root_dir, trajectories=trajectories, cameras=cameras)
    run_jobs(jobs, fps=30.0, overwrite=True, fourcc="mp4v", resize_to_first=True, verbose=True, parallel=True, max_workers=12)

    # Example 3) Everything under root (every folder literally named 'rgb')
    # jobs = make_jobs_for_all_rgb_dirs(root_dir)
    # run_jobs(jobs, fps=30.0, overwrite=False, parallel=True, max_workers=4)
