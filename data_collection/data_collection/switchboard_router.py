#!/usr/bin/env python3

import re
from dataclasses import dataclass
from typing import Optional

import serial

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool


DEFAULT_SWITCHBOARD_PORT = (
    "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_8553130333135180B2E1-if00"
)
DEFAULT_SWITCHBOARD_BAUDRATE = 115200

_EV_RE = re.compile(r"^EV:S(?P<switch>[1-5]):(?P<state>[01])$")
_STATE_RE = re.compile(r"^STATE:(?P<bits>[01]{5})$")


@dataclass(frozen=True)
class _GripperGoal:
    position: float
    max_effort: float


class SwitchboardRouter(Node):
    def __init__(self) -> None:
        super().__init__("switchboard_router")

        self.declare_parameter("port", DEFAULT_SWITCHBOARD_PORT)
        self.declare_parameter("baudrate", DEFAULT_SWITCHBOARD_BAUDRATE)
        self.declare_parameter("apply_startup_off", True)
        self.declare_parameter("startup_off_delay_s", 0.5)

        self.declare_parameter("blue_service", "/arduino_blue/set_enabled")
        self.declare_parameter("green_service", "/arduino_green/set_enabled")
        self.declare_parameter("recording_service", "/set_recording")
        self.declare_parameter("screen_recording_service", "/screen_recorder/set_recording")

        self.declare_parameter("gripper_action", "/robotiq_gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_open_position", 0.2)
        self.declare_parameter("gripper_close_position", 0.8)
        self.declare_parameter("gripper_max_effort", 50.0)

        self._port = str(self.get_parameter("port").value)
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._apply_startup_off = bool(self.get_parameter("apply_startup_off").value)
        self._startup_off_delay_s = float(self.get_parameter("startup_off_delay_s").value)

        self._blue_service_name = str(self.get_parameter("blue_service").value)
        self._green_service_name = str(self.get_parameter("green_service").value)
        self._recording_service_name = str(self.get_parameter("recording_service").value)
        self._screen_recording_service_name = str(self.get_parameter("screen_recording_service").value)

        self._gripper_action_name = str(self.get_parameter("gripper_action").value)
        open_pos = float(self.get_parameter("gripper_open_position").value)
        close_pos = float(self.get_parameter("gripper_close_position").value)
        max_eff = float(self.get_parameter("gripper_max_effort").value)
        self._gripper_open = _GripperGoal(position=open_pos, max_effort=max_eff)
        self._gripper_close = _GripperGoal(position=close_pos, max_effort=max_eff)

        self._serial: Optional[serial.Serial] = None
        self._rx_buf = bytearray()
        self._last_switch_state: dict[int, int] = {}

        self._blue_client = self.create_client(SetBool, self._blue_service_name) if self._blue_service_name else None
        self._green_client = self.create_client(SetBool, self._green_service_name) if self._green_service_name else None
        self._recording_client = (
            self.create_client(SetBool, self._recording_service_name) if self._recording_service_name else None
        )
        self._screen_recording_client = (
            self.create_client(SetBool, self._screen_recording_service_name)
            if self._screen_recording_service_name
            else None
        )
        self._gripper_client: ActionClient = ActionClient(self, GripperCommand, self._gripper_action_name)

        self._startup_sent_blue = False
        self._startup_sent_green = False
        self._startup_sent_gripper = False
        self._started_at = self.get_clock().now()
        self._startup_last_attempt_ns = 0

        self._serial_next_attempt_ns = 0
        self._serial_last_warn_ns = 0

        self.create_timer(0.02, self._tick)

        self.get_logger().info(
            "Switch mapping: S1->blue (Y/X), S2->green (Y/X), S3->gripper open/close, S4->start/stop all recording"
        )
        self.get_logger().info(f"Listening on {self._port} @ {self._baudrate}")

    def shutdown(self) -> None:
        try:
            if self._serial is not None:
                self._serial.close()
        except Exception:
            pass

    def _tick(self) -> None:
        self._ensure_serial()
        self._maybe_send_startup_off()
        self._read_serial_lines()

    def _ensure_serial(self) -> None:
        if self._serial is not None and self._serial.is_open:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self._serial_next_attempt_ns:
            return

        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0,
                write_timeout=0,
            )
            self._rx_buf.clear()
            self.get_logger().info("Connected to switchboard serial.")
        except Exception as exc:
            self._serial = None
            self._serial_next_attempt_ns = now_ns + int(2e9)
            if now_ns - self._serial_last_warn_ns > int(2e9):
                self._serial_last_warn_ns = now_ns
                self.get_logger().warning(f"Waiting for switchboard serial {self._port}: {exc}")

    def _maybe_send_startup_off(self) -> None:
        if not self._apply_startup_off:
            return

        now_ns = self.get_clock().now().nanoseconds
        if (self.get_clock().now() - self._started_at).nanoseconds < int(self._startup_off_delay_s * 1e9):
            return

        if now_ns - self._startup_last_attempt_ns < int(1e9):
            return
        self._startup_last_attempt_ns = now_ns

        if not self._startup_sent_blue:
            self._startup_sent_blue = self._call_set_bool(
                self._blue_client,
                self._blue_service_name,
                False,
                "startup S1=0 -> blue X",
            )

        if not self._startup_sent_green:
            self._startup_sent_green = self._call_set_bool(
                self._green_client,
                self._green_service_name,
                False,
                "startup S2=0 -> green X",
            )

        if not self._startup_sent_gripper:
            self._startup_sent_gripper = self._send_gripper_goal(self._gripper_close, "startup S3=0 -> gripper close")
        # Recording nodes may come and go between takes. Avoid spamming warnings by relying on
        # their default "off" state at startup; S4 edges will drive recording when services exist.

    def _read_serial_lines(self) -> None:
        if self._serial is None:
            return

        try:
            waiting = int(getattr(self._serial, "in_waiting", 0))
            data = self._serial.read(waiting or 1)
        except Exception as exc:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            self.get_logger().error(f"Lost switchboard serial: {exc}")
            return

        if not data:
            return

        self._rx_buf.extend(data)
        while b"\n" in self._rx_buf:
            raw_line, _, rest = self._rx_buf.partition(b"\n")
            self._rx_buf = bytearray(rest)

            line = raw_line.decode("utf-8", errors="ignore").strip()
            if line:
                self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        m_state = _STATE_RE.match(line)
        if m_state:
            bits = m_state.group("bits")
            self.get_logger().info(f"Switchboard state: {bits}")
            return

        m_ev = _EV_RE.match(line)
        if not m_ev:
            self.get_logger().debug(f"Ignoring line: {line!r}")
            return

        switch = int(m_ev.group("switch"))
        state = int(m_ev.group("state"))

        prev = self._last_switch_state.get(switch)
        self._last_switch_state[switch] = state
        if prev == state:
            return

        self.get_logger().info(f"Event: S{switch} -> {state}")

        if switch == 1:
            self._call_set_bool(self._blue_client, self._blue_service_name, bool(state), f"S1={state} -> blue")
        elif switch == 2:
            self._call_set_bool(self._green_client, self._green_service_name, bool(state), f"S2={state} -> green")
        elif switch == 3:
            self._send_gripper_goal(self._gripper_open if state else self._gripper_close, f"S3={state} -> gripper")
        elif switch == 4:
            enabled = bool(state)
            self._call_set_bool(
                self._recording_client,
                self._recording_service_name,
                enabled,
                f"S4={state} -> sensor recording",
            )
            self._call_set_bool(
                self._screen_recording_client,
                self._screen_recording_service_name,
                enabled,
                f"S4={state} -> screen recording",
            )

    def _call_set_bool(self, client, service_name: str, enabled: bool, label: str) -> bool:
        if not service_name or client is None:
            return True
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.0)

        if not client.service_is_ready():
            self.get_logger().warning(f"{label}: service not available: {service_name}")
            return False

        req = SetBool.Request()
        req.data = enabled

        fut = client.call_async(req)

        def _done(_fut) -> None:
            try:
                resp = _fut.result()
                self.get_logger().info(f"{label}: {resp.success} ({resp.message})")
            except Exception as exc:
                self.get_logger().error(f"{label}: failed: {exc}")

        fut.add_done_callback(_done)
        return True

    def _send_gripper_goal(self, goal: _GripperGoal, label: str) -> bool:
        if not self._gripper_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warning(f"{label}: gripper action not available: {self._gripper_action_name}")
            return False

        msg = GripperCommand.Goal()
        msg.command.position = float(goal.position)
        msg.command.max_effort = float(goal.max_effort)

        self.get_logger().info(f"{label}: sending GripperCommand position={msg.command.position}")
        self._gripper_client.send_goal_async(msg)
        return True


def main() -> int:
    rclpy.init()
    node = SwitchboardRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
