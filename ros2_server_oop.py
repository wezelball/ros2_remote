import socket
import json
import time
import threading
import cv2
from picamera2 import Picamera2

from base_ctrl import BaseController
from base_ctrl import ReadLine

HOST_IP = '0.0.0.0'
MOTOR_PORT = 5000
FEEDBACK_PORT = 6000
GIMBAL_PORT = 7000
VIDEO_PORT = 8000
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

class RoverController():
    def __init__(self):
        # Serial connection to the ESP32
        self.base = BaseController(SERIAL_PORT, BAUD_RATE)
        self.readline = ReadLine

        # Set up socket connections for different components
        self.motor_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.motor_sock.bind((HOST_IP, MOTOR_PORT))
        self.motor_sock.listen(1)

        self.feedback_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.feedback_sock.bind((HOST_IP, FEEDBACK_PORT))
        self.feedback_sock.listen(1)

        self.gimbal_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.gimbal_sock.bind((HOST_IP, GIMBAL_PORT))
        self.gimbal_sock.listen(1)

        self.video_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.video_sock.bind((HOST_IP, VIDEO_PORT))
        self.video_sock.listen(1)

        # Picamera2 for video streaming
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"format": "RGB888"}))
        self.picam2.start()

        # Start threads for motor, feedback, and video
        self.motor_thread = threading.Thread(target=self.motor_control_thread, daemon=True)
        self.motor_thread.start()

        self.feedback_thread = threading.Thread(target=self.feedback_thread_func, daemon=True)
        self.feedback_thread.start()

        self.gimbal_thread = threading.Thread(target=self.gimbal_control_thread, daemon=True)
        self.gimbal_thread.start()

        self.video_thread = threading.Thread(target=self.video_stream_thread, daemon=True)
        self.video_thread.start()

        print("Robot server has started")

    def send_serial_command(self, command):
        """Send JSON command via serial to the ESP32."""
        command_str = command
        self.base.send_command(command_str)
        #print(f'Sent command to ESP32: {command}')

    def motor_control_thread(self):
        """Thread to handle incoming commands (not just motors) from the laptop via sockets."""
        while True:
            conn, _ = self.motor_sock.accept()
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                try:
                    command = json.loads(data.decode('utf-8'))
                    self.send_serial_command(command)
                    #print(f'Received command: {command}')
                except json.JSONDecodeError:
                    print("Invalid JSON received")
            conn.close()

    def gimbal_control_thread(self):
        """Thread to handle incoming commands (not just motors) from the laptop via sockets."""
        while True:
            conn, _ = self.gimbal_sock.accept()
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                try:
                    command = json.loads(data.decode('utf-8'))
                    self.send_serial_command(command)
                    #print(f'Received command: {command}')
                except json.JSONDecodeError:
                    print("Invalid JSON received")
            conn.close()

    def receive_feedback_from_esp32(self):
        # Implement an infinite loop to continuously monitor serial port data.
        while True:
            try:
                data_recv_buffer = self.base.feedback_data()
                # Check if the parsed data contains the key 'T'.
                if 'T' in data_recv_buffer:
                    # If the value of 'T' is 1001, print the received data and break out of the loop.
                    if data_recv_buffer['T'] == 1001:
                        return data_recv_buffer
                        break
            # If an exception occurs while reading or processing the data, ignore the exception and continue to listen for the next line of data.
            except:
                print('receive feedback from esp32 failed')
                return None

    def feedback_thread_func(self):
        """Thread to handle feedback requests and responses."""
        while True:
            conn, _ = self.feedback_sock.accept()
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                command = json.loads(data.decode('utf-8'))
                #print(f'Feedback cmd from dev: {command} ')
                if command.get("T") == 130:
                    # Query for feedback
                    self.request_feedback()
                    feedback = self.receive_feedback_from_esp32()
                    #print(f'feedback_thread_func:feedback from esp32: {feedback}')
                    if feedback:
                        # Extract odometer and battery voltage from feedback
                        response = {
                            "odl": feedback.get('odl', 0),
                            "odr": feedback.get('odr', 0),
                            "battery_voltage": feedback.get('v', 0)
                        }
                        conn.sendall(json.dumps(response).encode('utf-8'))
            conn.close()

    def disable_auto_feedback(self):
        """Send command to disable automatic feedback from ESP32."""
        disable_command = {"T": 131, "cmd": 0}
        self.send_serial_command(disable_command)

    def request_feedback(self):
        """Request feedback from the ESP32."""
        feedback_command = {"T": 130}
        self.send_serial_command(feedback_command)

    def video_stream_thread(self):
        conn, addr = self.video_sock.accept()
        #conn, addr = s.accept()
        print(f'Video stream connection from {addr}')

        try:
            while True:
                frame = self.picam2.capture_array()

                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    continue

                # Send the length of the frame first
                frame_size = len(buffer)
                conn.sendall(frame_size.to_bytes(4, 'big'))

                # Send the actual frame
                conn.sendall(buffer.tobytes())

        except Exception as e:
            print(f'Error streaming video: {e}')

    # Gimbal control functions
    def control_gimbal_simple(self, x, y, spd=0, acc=0):
        """Basic gimbal control command."""
        command = {"T": 133, "X": x, "Y": y, "SPD": spd, "ACC": acc}
        self.send_serial_command(command)

    def control_gimbal_move(self, x, y, sx, sy):
        """Continuous gimbal control command."""
        command = {"T": 134, "X": x, "Y": y, "SX": sx, "SY": sy}
        self.send_serial_command(command)

    def stop_gimbal(self):
        """Stop gimbal movement."""
        command = {"T": 135}
        self.send_serial_command(command)

    def close_sockets(self):
        """Close network sockets on shutdown"""
        self.motor_sock.close()
        self.feedback_sock.close()
        self.video_sock.close()


# Example usage
if __name__ == '__main__':
    rover = RoverController()
    rover.disable_auto_feedback()

    try:
        while True:
            time.sleep(1)
            # Place logging/status checks here
    except KeyboardInterrupt:
        print("Shutting down RoverController")
        rover.close_sockets()

