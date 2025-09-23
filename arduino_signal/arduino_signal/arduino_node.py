import rclpy
from rclpy.node import Node
import serial
import time

# class ArduinoNode(Node):
#     def __init__(self):
#         super().__init__('arduino_node')
#         self.ser = serial.Serial('/dev/ttyACM0', 9600)
#         self.ser.write(b'1')  # Turn on LED
#         print('LED should be ON.')
#         time.sleep(5)
#         self.ser.write(b'0')
#         print('LED should be OFF.')
#         self.ser.close()


ser = serial.Serial('/dev/ttyACM0', 1000000)
time.sleep(2)
ser.write(b'1')  # Turn on LED
print('LED should be ON.')
time.sleep(5)
ser.write(b'0')
print('LED should be OFF.')
ser.close()

# try:
#     ser = serial.Serial('/dev/ttyACM0', 9600)
#     time.sleep(2)  # Wait for Arduino to reset
#     ser.write(b'1')  # Turn on LED
#     print('LED should be ON.')
#     time.sleep(5)  # Keep it on for 5 seconds
#     ser.write(b'0')  # Turn off LED
#     print('LED should be OFF.')
# except serial.SerialException as e:
#     print(f'Error: {e}')
# finally:
#     ser.close()

# def main(args=None):
#     rclpy.init(args=args)
#     arduino_node = ArduinoNode()
#     rclpy.spin(arduino_node)
#     arduino_node.destroy_node()
#     rclpy.shutdown()
