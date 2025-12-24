#!/usr/bin/env python3

import os
import re
import shutil
import signal
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


_TAKE_ENV_NAME = "SERVICE_LAUNCH_TAKE_NAME"
_TAKE_ENV_DIR = "SERVICE_LAUNCH_TAKE_DIR"


@dataclass(frozen=True)
class _X11Geometry:
    width: int
    height: int
    x: int
    y: int


_XRANDR_MON_RE = re.compile(
    r"^\s*\d+:\s+(?P<flags>[+*]+)(?P<name>\S+)\s+"
    r"(?P<w>\d+)/\d+x(?P<h>\d+)/\d+\+(?P<x>\d+)\+(?P<y>\d+)\s+"
)


def _parse_geometry(value: str) -> Optional[_X11Geometry]:
    """
    Accepts:
      - "WIDTHxHEIGHT+X+Y" (xrandr-style)
      - "WIDTHxHEIGHT+X,Y" (ffmpeg-style)
    """
    v = value.strip()
    if not v:
        return None

    m = re.match(r"^(?P<w>\d+)x(?P<h>\d+)\+(?P<x>\d+)[+,](?P<y>\d+)$", v)
    if not m:
        return None
    return _X11Geometry(
        width=int(m.group("w")),
        height=int(m.group("h")),
        x=int(m.group("x")),
        y=int(m.group("y")),
    )


def _x11_primary_monitor_geometry(monitor_name: str = "") -> Optional[Tuple[str, _X11Geometry]]:
    if not shutil.which("xrandr"):
        return None

    try:
        out = subprocess.check_output(["xrandr", "--listmonitors"], text=True, stderr=subprocess.STDOUT)
    except Exception:
        return None

    monitors: list[tuple[str, str, _X11Geometry]] = []
    for line in out.splitlines():
        m = _XRANDR_MON_RE.match(line)
        if not m:
            continue
        name = m.group("name")
        flags = m.group("flags")
        geom = _X11Geometry(
            width=int(m.group("w")),
            height=int(m.group("h")),
            x=int(m.group("x")),
            y=int(m.group("y")),
        )
        monitors.append((name, flags, geom))

    if not monitors:
        return None

    if monitor_name.strip():
        wanted = monitor_name.strip()
        for name, flags, geom in monitors:
            if name == wanted:
                return name, geom
        return None

    # Prefer primary (flag contains '*'), else first.
    for name, flags, geom in monitors:
        if "*" in flags:
            return name, geom
    name, _flags, geom = monitors[0]
    return name, geom


class ScreenRecorder(Node):
    def __init__(self) -> None:
        super().__init__("screen_recorder")

        self.declare_parameter("backend", "auto")  # auto|ffmpeg_x11|wf_recorder
        self.declare_parameter("monitor", "")  # X11: xrandr monitor name, default=primary
        self.declare_parameter("geometry", "")  # X11: WIDTHxHEIGHT+X+Y (or +X,Y)
        self.declare_parameter("display", "")  # X11: e.g. ":0.0" (default from $DISPLAY)
        self.declare_parameter("wayland_output", "")  # Wayland: output name for wf-recorder (-o)

        # ffmpeg encoder settings (X11 backend)
        self.declare_parameter("codec", "libx264")
        self.declare_parameter("pix_fmt", "yuv420p")  # try yuv444p for sharper text (less compatible)
        self.declare_parameter("profile", "")  # e.g. high, high444
        self.declare_parameter("tune", "")  # e.g. stillimage, animation
        self.declare_parameter("x264_params", "")  # extra encoder params (e.g. "keyint=60:scenecut=0")
        self.declare_parameter("bitrate_kbps", 0)  # 0 = CRF mode
        self.declare_parameter("thread_queue_size", 1024)  # larger helps avoid frame drops if encoder lags
        self.declare_parameter("rtbufsize", "512M")  # capture buffer, larger helps smoothness
        self.declare_parameter("vsync", "cfr")  # ffmpeg: cfr|vfr|passthrough

        self.declare_parameter("fps", 30)
        self.declare_parameter("crf", 23)
        self.declare_parameter("preset", "veryfast")

        self.declare_parameter("output_dir", "")  # default from $SERVICE_LAUNCH_TAKE_DIR
        self.declare_parameter("filename_prefix", "")  # default from $SERVICE_LAUNCH_TAKE_NAME
        self.declare_parameter("filename", "")  # optional explicit filename (within output_dir)

        self._backend = str(self.get_parameter("backend").value).strip()
        self._monitor = str(self.get_parameter("monitor").value).strip()
        self._geometry = str(self.get_parameter("geometry").value).strip()
        self._display = str(self.get_parameter("display").value).strip()
        self._wayland_output = str(self.get_parameter("wayland_output").value).strip()

        self._codec = str(self.get_parameter("codec").value).strip()
        self._pix_fmt = str(self.get_parameter("pix_fmt").value).strip()
        self._profile = str(self.get_parameter("profile").value).strip()
        self._tune = str(self.get_parameter("tune").value).strip()
        self._x264_params = str(self.get_parameter("x264_params").value).strip()
        self._bitrate_kbps = int(self.get_parameter("bitrate_kbps").value)
        self._thread_queue_size = int(self.get_parameter("thread_queue_size").value)
        self._rtbufsize = str(self.get_parameter("rtbufsize").value).strip()
        self._vsync = str(self.get_parameter("vsync").value).strip() or "cfr"

        self._fps = int(self.get_parameter("fps").value)
        self._crf = int(self.get_parameter("crf").value)
        self._preset = str(self.get_parameter("preset").value).strip()

        self._output_dir = str(self.get_parameter("output_dir").value).strip()
        self._filename_prefix = str(self.get_parameter("filename_prefix").value).strip()
        self._filename = str(self.get_parameter("filename").value).strip()

        self._proc: Optional[subprocess.Popen] = None
        self._current_path: Optional[Path] = None

        self.create_service(SetBool, "~/set_recording", self._on_set_recording)
        self.get_logger().info("Service: /screen_recorder/set_recording (std_srvs/SetBool)")

    def _resolve_output_dir(self) -> Path:
        out = self._output_dir or os.environ.get(_TAKE_ENV_DIR, "").strip()
        if not out:
            out = os.path.join(os.path.expanduser("~"), "Videos")
        p = Path(out).expanduser()
        p.mkdir(parents=True, exist_ok=True)
        return p

    def _resolve_prefix(self) -> str:
        prefix = self._filename_prefix or os.environ.get(_TAKE_ENV_NAME, "").strip()
        return prefix or "screen"

    def _allocate_output_path(self, out_dir: Path) -> Path:
        """
        Default behavior: create numbered segments so pause/resume never overwrites:
          ft_058_1.mp4, ft_058_2.mp4, ...
        """
        if self._filename:
            candidate = (out_dir / self._filename).expanduser()
            if candidate.suffix.lower() != ".mp4":
                candidate = candidate.with_suffix(".mp4")
            if not candidate.exists():
                return candidate

            stem = candidate.stem
            ext = candidate.suffix
            seg_re = re.compile(rf"^{re.escape(stem)}_(?P<idx>\d+){re.escape(ext)}$")
            max_idx = 0
            for path in out_dir.glob(f"{stem}_*{ext}"):
                m = seg_re.match(path.name)
                if not m:
                    continue
                max_idx = max(max_idx, int(m.group("idx")))
            return out_dir / f"{stem}_{max_idx + 1}{ext}"

        prefix = self._resolve_prefix()
        seg_re = re.compile(rf"^{re.escape(prefix)}_(?P<idx>\d+)\.mp4$")
        max_idx = 0
        for path in out_dir.glob(f"{prefix}_*.mp4"):
            m = seg_re.match(path.name)
            if not m:
                continue
            max_idx = max(max_idx, int(m.group("idx")))
        return out_dir / f"{prefix}_{max_idx + 1}.mp4"

    def _choose_backend(self) -> str:
        if self._backend and self._backend != "auto":
            return self._backend

        if os.environ.get("XDG_SESSION_TYPE", "").lower() == "wayland" and shutil.which("wf-recorder"):
            return "wf_recorder"
        if os.environ.get("DISPLAY") and shutil.which("ffmpeg"):
            return "ffmpeg_x11"
        if shutil.which("ffmpeg") and os.environ.get("DISPLAY"):
            return "ffmpeg_x11"
        return "unknown"

    def _ffmpeg_cmd(self, output_path: Path) -> list[str]:
        if not shutil.which("ffmpeg"):
            raise RuntimeError("ffmpeg not found in PATH")

        display = self._display or os.environ.get("DISPLAY", "").strip()
        if not display:
            raise RuntimeError("DISPLAY is not set (are you on X11?)")

        geom = _parse_geometry(self._geometry)
        selected_monitor = ""
        if geom is None:
            resolved = _x11_primary_monitor_geometry(self._monitor)
            if resolved is None:
                raise RuntimeError(
                    "Could not resolve X11 monitor geometry. Set parameter 'geometry' "
                    "to e.g. '1920x1080+0+0' (from `xrandr --listmonitors`)."
                )
            selected_monitor, geom = resolved

        if selected_monitor:
            self.get_logger().info(
                f"X11 capture monitor={selected_monitor} size={geom.width}x{geom.height} offset=+{geom.x}+{geom.y}"
            )
        else:
            self.get_logger().info(
                f"X11 capture size={geom.width}x{geom.height} offset=+{geom.x}+{geom.y}"
            )

        input_spec = f"{display}+{geom.x},{geom.y}"
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-y",
            "-thread_queue_size",
            str(max(64, self._thread_queue_size)),
            "-f",
            "x11grab",
            "-rtbufsize",
            (self._rtbufsize or "512M"),
            "-draw_mouse",
            "1",
            "-framerate",
            str(self._fps),
            "-video_size",
            f"{geom.width}x{geom.height}",
            "-i",
            input_spec,
            "-c:v",
            (self._codec or "libx264"),
            "-preset",
            self._preset,
        ]
        if self._bitrate_kbps > 0:
            kbps = int(self._bitrate_kbps)
            cmd.extend(["-b:v", f"{kbps}k", "-maxrate", f"{kbps}k", "-bufsize", f"{kbps*2}k"])
        else:
            cmd.extend(["-crf", str(self._crf)])

        if self._tune:
            cmd.extend(["-tune", self._tune])
        if self._profile:
            cmd.extend(["-profile:v", self._profile])
        if self._x264_params and (self._codec or "libx264") in ("libx264", "libx264rgb"):
            cmd.extend(["-x264-params", self._x264_params])

        if self._vsync:
            cmd.extend(["-vsync", self._vsync])
            if self._vsync == "cfr":
                cmd.extend(["-r", str(self._fps)])

        cmd.extend(["-pix_fmt", (self._pix_fmt or "yuv420p"), str(output_path)])
        return cmd

    def _wf_recorder_cmd(self, output_path: Path) -> list[str]:
        if not shutil.which("wf-recorder"):
            raise RuntimeError("wf-recorder not found in PATH")

        cmd = ["wf-recorder", "-f", str(output_path), "-r", str(self._fps)]
        if self._wayland_output:
            cmd.extend(["-o", self._wayland_output])
        # geometry is optional on wf-recorder, but we accept it if provided.
        if self._geometry.strip():
            cmd.extend(["-g", self._geometry.strip()])
        return cmd

    def _start(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            raise RuntimeError(f"Already recording: {self._current_path}")

        out_dir = self._resolve_output_dir()
        output_path = self._allocate_output_path(out_dir)

        backend = self._choose_backend()
        if backend == "ffmpeg_x11":
            cmd = self._ffmpeg_cmd(output_path)
        elif backend == "wf_recorder":
            cmd = self._wf_recorder_cmd(output_path)
        else:
            raise RuntimeError(
                "No supported screen recording backend found. "
                "Install `ffmpeg` (X11) or `wf-recorder` (Wayland)."
            )

        self.get_logger().info(f"Starting screen recording: {' '.join(cmd)}")
        self._proc = subprocess.Popen(cmd)
        self._current_path = output_path
        self.get_logger().info(f"Recording to: {output_path}")

    def _stop(self) -> None:
        if self._proc is None:
            return
        if self._proc.poll() is not None:
            self._proc = None
            self._current_path = None
            return

        self.get_logger().info("Stopping screen recording...")
        try:
            self._proc.send_signal(signal.SIGINT)
            self._proc.wait(timeout=5.0)
        except Exception:
            try:
                self._proc.kill()
            except Exception:
                pass
        finally:
            path = self._current_path
            self._proc = None
            self._current_path = None
            if path is not None:
                self.get_logger().info(f"Saved: {path}")

    def _on_set_recording(self, req: SetBool.Request, resp: SetBool.Response) -> SetBool.Response:
        try:
            if req.data:
                self._start()
                resp.success = True
                resp.message = "recording started"
            else:
                self._stop()
                resp.success = True
                resp.message = "recording stopped"
        except Exception as exc:
            resp.success = False
            resp.message = str(exc)
            self.get_logger().error(resp.message)
        return resp


def main(argv=None) -> int:
    full_argv = sys.argv if argv is None else argv
    rclpy.init(args=full_argv)
    node = ScreenRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
