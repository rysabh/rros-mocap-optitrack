import serial
import time
import signal
import sys
import pyrealsense2 as rs
import numpy as np
import cv2
import threading

# Set up the serial connection with Arduino
arduino = serial.Serial(port='/dev/ttyACM3', baudrate=9600, timeout=1)

def send_command(command):  
    print(f"Signal received, sending {command} to microcontroller")
    arduino.write(command.encode())  # Send the command to the Arduino
    time.sleep(0.05)  # Small delay to ensure the command is sent properly

def signal_handler(sig, frame):
    print("Signal received, sending 'X' to microcontroller...")
    send_command('X')  # Send the 'X' command when the process is terminated
    sys.exit(0)  # Exit the program

# Register the signal handler for termination signals   
signal.signal(signal.SIGINT, signal_handler)  # Handles Ctrl+C
signal.signal(signal.SIGTERM, signal_handler)  # Handles termination signal

# Function to run the RealSense camera stream in a separate thread
def start_realsense_stream():
# Create two pipelines for two cameras
    pipeline1 = rs.pipeline()
    # pipeline2 = rs.pipeline()

    # Configure the first pipeline (camera 1)
    config1 = rs.config()
    config1.enable_device('038522062901')  # Replace with the actual serial number of your first camera
    config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Adjust stream parameters as needed
    config1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 30 FPS depth stream


    # Configure the second pipeline (camera 2)
    # config2 = rs.config()
    # config2.enable_device('932122060300')  # Replace with the serial number of your second camera
    # config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Adjust stream parameters as needed
    # config2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 30 FPS depth stream


    # Start the pipelines
    pipeline1.start(config1)
    # pipeline2.start(config2)

    try:
        while True:
            # Wait for frames from both cameras
            frames1 = pipeline1.wait_for_frames()
            # frames2 = pipeline2.wait_for_frames()

            # Get color frames
            color_frame1 = frames1.get_color_frame()
            depth_frame1 = frames1.get_depth_frame()

            # color_frame2 = frames2.get_color_frame()
            # depth_frame2 = frames2.get_depth_frame()


            # Convert images to numpy arrays
            color_image1 = np.asanyarray(color_frame1.get_data())
            # color_image2 = np.asanyarray(color_frame2.get_data())

            if not color_frame1 or not depth_frame1:
                continue  # Skip if frames are not ready

            # if not color_frame2 or not depth_frame2:
            #     continue

            depth_image1 = np.asanyarray(depth_frame1.get_data())
            color_image1 = np.asanyarray(color_frame1.get_data())
            depth_colormap1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image1, alpha=0.03), cv2.COLORMAP_JET)

            # depth_image2 = np.asanyarray(depth_frame2.get_data())
            # color_image2 = np.asanyarray(color_frame2.get_data())
            # depth_colormap2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image2, alpha=0.03), cv2.COLORMAP_JET)

            images = np.hstack((color_image1)) #, color_image2))
            images = color_image1
            cv2.imshow('RealSense Stream (World + Robot)', images)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline1.stop()
        # pipeline2.stop()
        cv2.destroyAllWindows()


def visual_timer(seconds):
    for remaining in range(seconds, 0, -1):
        print(f"Timer: {remaining} second{'s' if remaining > 1 else ''} remaining", end='\r')
        time.sleep(1)
    print("\nTimer complete!")


def handle_user_input():
    while True:
        command = input("Enter command (X: Press TO STOP, Y Press TO START) \n Z (10,10) or L (10,20) or Q (5,5): ").strip().upper()
        if command == 'X':
            send_command('X')

        elif command == 'Y':
            send_command('Y')

        elif command == 'Z':
            visual_timer(10)
            send_command('Y')
            visual_timer(10)
            send_command('X')

        elif command == 'L':
            visual_timer(10)
            send_command('Y')
            visual_timer(20)
            send_command('X')

        elif command == 'Q':
            visual_timer(5)
            send_command('Y')
            visual_timer(5)
            send_command('X')

        else:
            print("Invalid command. Please enter X or Y or Z (10,10) or L (10,20) or Q (5,5).")
            
# Start the RealSense stream in a separate thread
# realsense_thread = threading.Thread(target=start_realsense_stream, daemon=True)
# realsense_thread.start()

# Run the user input function in the main thread
handle_user_input()