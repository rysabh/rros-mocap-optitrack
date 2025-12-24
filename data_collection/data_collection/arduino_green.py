#!/usr/bin/env python3

import argparse
import sys
import threading
import time
from typing import Optional

import serial

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import SetBool


DEFAULT_PORT = (
    "/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_95530343834351B090F1-if00"
)
DEFAULT_BAUDRATE = 9600


def visual_timer(seconds: int) -> None:
    for remaining in range(seconds, 0, -1):
        print(
            f"Timer: {remaining} second{'s' if remaining > 1 else ''} remaining",
            end="\r",
            flush=True,
        )
        time.sleep(1)
    print("\nTimer complete!", flush=True)


class ArduinoGreenNode(Node):
    def __init__(self, *, port: str, baudrate: int, startup_off: bool) -> None:
        super().__init__("arduino_green")

        self._lock = threading.Lock()
        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=1)

        self._srv = self.create_service(SetBool, "~/set_enabled", self._on_set_enabled)

        self.get_logger().info(f"Connected: port={port} baudrate={baudrate}")
        self.get_logger().info("Service: /arduino_green/set_enabled (std_srvs/SetBool)")

        if startup_off:
            self.send_command("X")

    def send_command(self, command: str) -> None:
        command = (command or "").strip().upper()
        if len(command) != 1:
            raise ValueError(f"Expected a 1-char command, got: {command!r}")

        with self._lock:
            self._serial.write(command.encode("utf-8"))
            self._serial.flush()

        self.get_logger().info(f"Sent '{command}' to green Arduino")
        time.sleep(0.05)

    def _on_set_enabled(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        try:
            self.send_command("Y" if request.data else "X")
            response.success = True
            response.message = "ok"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def shutdown(self) -> None:
        try:
            self.send_command("X")
        except Exception:
            pass
        try:
            self._serial.close()
        except Exception:
            pass


def _interactive_loop(node: ArduinoGreenNode) -> None:
    while rclpy.ok():
        try:
            command = input(
                "Green Arduino cmd (X stop, Y start, Z/L/Q timed): "
            ).strip().upper()
        except EOFError:
            return
        except KeyboardInterrupt:
            return

        if not command:
            continue

        try:
            if command == "X":
                node.send_command("X")
            elif command == "Y":
                visual_timer(2)
                node.send_command("Y")
            elif command == "Z":
                visual_timer(10)
                node.send_command("Y")
                visual_timer(10)
                node.send_command("X")
            elif command == "L":
                visual_timer(10)
                node.send_command("Y")
                visual_timer(20)
                node.send_command("X")
            elif command == "Q":
                visual_timer(5)
                node.send_command("Y")
                visual_timer(5)
                node.send_command("X")
            else:
                print("Invalid. Use X, Y, Z, L, or Q.", flush=True)
        except Exception as exc:
            print(f"Failed to send command: {exc}", flush=True)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Green Arduino controller (ROS2 service + optional CLI).")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument(
        "--no-interactive",
        action="store_true",
        help="Disable stdin prompt (recommended when driven by the switchboard).",
    )
    parser.add_argument(
        "--startup-off",
        action="store_true",
        default=True,
        help="Send 'X' once on startup (default).",
    )
    parser.add_argument(
        "--no-startup-off",
        dest="startup_off",
        action="store_false",
        help="Do not send 'X' on startup.",
    )

    argv = argv if argv is not None else sys.argv
    rclpy.init(args=argv)
    args = parser.parse_args(remove_ros_args(argv)[1:])
    node = ArduinoGreenNode(port=args.port, baudrate=args.baudrate, startup_off=args.startup_off)

    if not args.no_interactive:
        threading.Thread(target=_interactive_loop, args=(node,), daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().exception("arduino_green crashed")
        return 1
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
