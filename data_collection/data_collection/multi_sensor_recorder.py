import csv
import os
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import WrenchStamped
from mocap_optitrack_interfaces.msg import MotionCaptureData
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py import message_to_yaml
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool, Trigger


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass
class _StampedMsg:
    stamp_ns: int
    msg: Any


class _StampedBuffer:
    def __init__(self, maxlen: int) -> None:
        self._buf: Deque[_StampedMsg] = deque(maxlen=maxlen)

    def add(self, stamp_ns: int, msg: Any) -> None:
        self._buf.append(_StampedMsg(stamp_ns=stamp_ns, msg=msg))

    def closest(self, target_ns: int, max_delta_ns: int) -> Optional[_StampedMsg]:
        if not self._buf:
            return None

        best: Optional[_StampedMsg] = None
        best_delta = max_delta_ns + 1
        for item in self._buf:
            delta = abs(item.stamp_ns - target_ns)
            if delta < best_delta:
                best = item
                best_delta = delta

        return best if best is not None and best_delta <= max_delta_ns else None


class MultiSensorRecorder(Node):
    def __init__(self) -> None:
        super().__init__("multi_sensor_recorder")

        self.declare_parameter("output_dir", "")
        self.declare_parameter("take_name", "")
        self.declare_parameter("record_hz", 30.0)
        self.declare_parameter("record_enabled", True)
        self.declare_parameter("max_sync_slop_ms", 50.0)
        self.declare_parameter("mocap_topic", "mocap_Data")
        self.declare_parameter("wrench_topic", "wrench")
        self.declare_parameter("enable_cameras", True)
        self.declare_parameter("camera_names", ["camera_1", "camera_2", "camera_3", "camera_4"])

        output_dir = str(self.get_parameter("output_dir").value).strip()
        take_name = str(self.get_parameter("take_name").value).strip() or "take"
        record_hz = float(self.get_parameter("record_hz").value)
        self._record_enabled = bool(self.get_parameter("record_enabled").value)
        self._max_sync_slop_ns = int(float(self.get_parameter("max_sync_slop_ms").value) * 1e6)

        self._mocap_topic = str(self.get_parameter("mocap_topic").value).strip()
        self._wrench_topic = str(self.get_parameter("wrench_topic").value).strip()
        self._enable_cameras = bool(self.get_parameter("enable_cameras").value)
        self._camera_names: List[str] = list(self.get_parameter("camera_names").value)

        if not output_dir:
            output_dir = os.path.join(os.path.expanduser("~"), "multi_iiwa_ws_data", "takes", take_name)

        self._take_dir = Path(output_dir)
        self._take_dir.mkdir(parents=True, exist_ok=True)
        self._take_name = take_name

        self._bridge = CvBridge()
        self._sample_id = 0
        self._fieldnames = self._build_fieldnames(self._camera_names)
        self._segment_index = 0
        self._segment_start_ns: Optional[int] = None
        self._samples_csv_file: Optional[Any] = None
        self._csv_writer: Optional[csv.DictWriter] = None

        # Only create the first CSV file when recording actually starts (S4=1),
        # so each pause/resume produces <take>_1.csv, <take>_2.csv, ...
        if self._record_enabled:
            self._start_new_segment()

        self._mocap_dir = self._take_dir / "mocap"
        self._mocap_dir.mkdir(parents=True, exist_ok=True)

        self._cameras_dir = self._take_dir / "cameras"
        self._cameras_dir.mkdir(parents=True, exist_ok=True)

        self._mocap_buf = _StampedBuffer(maxlen=400)
        self._wrench_buf = _StampedBuffer(maxlen=800)

        self._rgb_raw_buf: Dict[str, _StampedBuffer] = {}
        self._rgb_rect_buf: Dict[str, _StampedBuffer] = {}
        self._depth_buf: Dict[str, _StampedBuffer] = {}
        self._rgb_info: Dict[str, CameraInfo] = {}
        self._depth_info: Dict[str, CameraInfo] = {}
        self._camera_info_written: Dict[str, bool] = {}

        self._mocap_sub = self.create_subscription(
            MotionCaptureData, self._mocap_topic, self._on_mocap, 10
        )
        self._wrench_sub = self.create_subscription(
            WrenchStamped, self._wrench_topic, self._on_wrench, 10
        )

        if self._enable_cameras:
            for cam in self._camera_names:
                self._rgb_raw_buf[cam] = _StampedBuffer(maxlen=60)
                self._rgb_rect_buf[cam] = _StampedBuffer(maxlen=60)
                self._depth_buf[cam] = _StampedBuffer(maxlen=60)
                self._camera_info_written[cam] = False

                rgb_topic = f"/{cam}/camera/color/image_raw"
                rgb_rect_topic = f"/{cam}/camera/color/image_rect_raw"
                depth_topic = f"/{cam}/camera/aligned_depth_to_color/image_raw"
                rgb_info_topic = f"/{cam}/camera/color/camera_info"
                depth_info_topic = f"/{cam}/camera/aligned_depth_to_color/camera_info"

                self.create_subscription(
                    Image,
                    rgb_topic,
                    lambda msg, cam=cam: self._on_rgb_raw(cam, msg),
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    Image,
                    rgb_rect_topic,
                    lambda msg, cam=cam: self._on_rgb_rect(cam, msg),
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    Image,
                    depth_topic,
                    lambda msg, cam=cam: self._on_depth(cam, msg),
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    CameraInfo,
                    rgb_info_topic,
                    lambda msg, cam=cam: self._on_rgb_info(cam, msg),
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    CameraInfo,
                    depth_info_topic,
                    lambda msg, cam=cam: self._on_depth_info(cam, msg),
                    qos_profile_sensor_data,
                )

        self._set_recording_srv = self.create_service(SetBool, "set_recording", self._set_recording)
        self._capture_sample_srv = self.create_service(Trigger, "capture_sample", self._capture_sample_srv_cb)

        self._timer = self.create_timer(1.0 / max(1e-3, record_hz), self._timer_cb)

        self.get_logger().info(f"Recording take '{take_name}' to {str(self._take_dir)}")

    def _start_new_segment(self) -> None:
        self._close_csv()

        self._segment_index += 1
        self._segment_start_ns = self.get_clock().now().nanoseconds

        csv_path = self._take_dir / f"{self._take_name}_{self._segment_index}.csv"
        self._samples_csv_file = open(csv_path, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._samples_csv_file, fieldnames=self._fieldnames)
        self._csv_writer.writeheader()
        self._samples_csv_file.flush()
        self.get_logger().info(f"CSV segment: {csv_path.name}")

    def _close_csv(self) -> None:
        if self._samples_csv_file is None:
            return
        try:
            self._samples_csv_file.close()
        finally:
            self._samples_csv_file = None
            self._csv_writer = None

    @staticmethod
    def _build_fieldnames(camera_names: List[str]) -> List[str]:
        fields = [
            "sample_id",
            "time_s",
            "sample_time_ns",
            "mocap_stamp_ns",
            "mocap_yaml_path",
            "ati_stamp_ns",
            "fx",
            "fy",
            "fz",
            "tx",
            "ty",
            "tz",
        ]
        for cam in camera_names:
            fields.extend(
                [
                    f"{cam}_rgb_stamp_ns",
                    f"{cam}_rgb_path",
                    f"{cam}_depth_stamp_ns",
                    f"{cam}_depth_path",
                ]
            )
        return fields

    def _on_mocap(self, msg: MotionCaptureData) -> None:
        self._mocap_buf.add(_stamp_to_ns(msg.header.stamp), msg)

    def _on_wrench(self, msg: WrenchStamped) -> None:
        self._wrench_buf.add(_stamp_to_ns(msg.header.stamp), msg)

    def _on_rgb_raw(self, cam: str, msg: Image) -> None:
        self._rgb_raw_buf[cam].add(_stamp_to_ns(msg.header.stamp), msg)

    def _on_rgb_rect(self, cam: str, msg: Image) -> None:
        self._rgb_rect_buf[cam].add(_stamp_to_ns(msg.header.stamp), msg)

    def _on_depth(self, cam: str, msg: Image) -> None:
        self._depth_buf[cam].add(_stamp_to_ns(msg.header.stamp), msg)

    def _on_rgb_info(self, cam: str, msg: CameraInfo) -> None:
        self._rgb_info[cam] = msg

    def _on_depth_info(self, cam: str, msg: CameraInfo) -> None:
        self._depth_info[cam] = msg

    def _timer_cb(self) -> None:
        if not self._record_enabled:
            return
        self._capture_sample()

    def _set_recording(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        desired = bool(request.data)

        # On every start edge, begin a new CSV segment so pause/resume doesn't append.
        if desired and not self._record_enabled:
            self._start_new_segment()
        if not desired and self._record_enabled:
            self._close_csv()

        self._record_enabled = desired
        response.success = True
        response.message = f"record_enabled={self._record_enabled} segment={self._segment_index}"
        return response

    def _capture_sample_srv_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        sample_id = self._capture_sample()
        response.success = sample_id is not None
        response.message = "" if sample_id is None else f"captured sample_id={sample_id}"
        return response

    def _write_camera_info_if_ready(self, cam: str) -> None:
        if self._camera_info_written.get(cam, False):
            return

        rgb_info = self._rgb_info.get(cam)
        depth_info = self._depth_info.get(cam)
        if rgb_info is None or depth_info is None:
            return

        cam_dir = self._cameras_dir / cam
        cam_dir.mkdir(parents=True, exist_ok=True)

        (cam_dir / "camera_info_color.yaml").write_text(message_to_yaml(rgb_info), encoding="utf-8")
        (cam_dir / "camera_info_depth.yaml").write_text(message_to_yaml(depth_info), encoding="utf-8")
        self._camera_info_written[cam] = True

    def _save_image(self, path: Path, msg: Image, encoding: str) -> None:
        cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), cv_img)

    def _capture_sample(self) -> Optional[int]:
        target_ns = self.get_clock().now().nanoseconds
        self._sample_id += 1
        sample_id = self._sample_id

        if self._csv_writer is None:
            self._start_new_segment()

        row: Dict[str, Any] = {name: "" for name in self._fieldnames}
        row["sample_id"] = sample_id
        seg_start = self._segment_start_ns or target_ns
        row["time_s"] = f"{(target_ns - seg_start) / 1e9:.6f}"
        row["sample_time_ns"] = target_ns

        mocap_item = self._mocap_buf.closest(target_ns, self._max_sync_slop_ns)
        if mocap_item is not None:
            row["mocap_stamp_ns"] = mocap_item.stamp_ns
            mocap_path = self._mocap_dir / f"sample_{sample_id:06d}.yaml"
            mocap_path.write_text(message_to_yaml(mocap_item.msg), encoding="utf-8")
            row["mocap_yaml_path"] = str(mocap_path.relative_to(self._take_dir))

        wrench_item = self._wrench_buf.closest(target_ns, self._max_sync_slop_ns)
        if wrench_item is not None:
            row["ati_stamp_ns"] = wrench_item.stamp_ns
            wrench: WrenchStamped = wrench_item.msg
            row["fx"] = wrench.wrench.force.x
            row["fy"] = wrench.wrench.force.y
            row["fz"] = wrench.wrench.force.z
            row["tx"] = wrench.wrench.torque.x
            row["ty"] = wrench.wrench.torque.y
            row["tz"] = wrench.wrench.torque.z

        if self._enable_cameras:
            for cam in self._camera_names:
                self._write_camera_info_if_ready(cam)

                rgb_item = self._rgb_raw_buf[cam].closest(target_ns, self._max_sync_slop_ns)
                if rgb_item is None:
                    rgb_item = self._rgb_rect_buf[cam].closest(target_ns, self._max_sync_slop_ns)
                if rgb_item is not None:
                    row[f"{cam}_rgb_stamp_ns"] = rgb_item.stamp_ns
                    rgb_path = Path("cameras") / cam / "rgb" / f"rgb_{sample_id:06d}.png"
                    self._save_image(self._take_dir / rgb_path, rgb_item.msg, encoding="bgr8")
                    row[f"{cam}_rgb_path"] = str(rgb_path)

                depth_item = self._depth_buf[cam].closest(target_ns, self._max_sync_slop_ns)
                if depth_item is not None:
                    row[f"{cam}_depth_stamp_ns"] = depth_item.stamp_ns
                    depth_path = Path("cameras") / cam / "depth" / f"depth_{sample_id:06d}.png"
                    self._save_image(self._take_dir / depth_path, depth_item.msg, encoding="passthrough")
                    row[f"{cam}_depth_path"] = str(depth_path)

        if self._csv_writer is not None and self._samples_csv_file is not None:
            self._csv_writer.writerow(row)
            self._samples_csv_file.flush()
        return sample_id

    def destroy_node(self) -> bool:
        try:
            self._close_csv()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiSensorRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
