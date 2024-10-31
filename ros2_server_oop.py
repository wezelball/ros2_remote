import socket
import serial
import json
import time
from queue import PriorityQueue
import threading
import cv2
from picamera2 import Picamera2

HOST_IP = '0.0.0.0'
MOTOR_PORT = 5000
LIGHTS_PORT = 5500
FEEDBACK_PORT = 6000
GIMBAL_PORT = 7000
VIDEO_PORT = 8000
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

class RoverController():
    def __init__(self):
        # Initialize priority queue and lock for serial access
        self.command_queue = PriorityQueue()
        self.serial_lock = threading.Lock()

        # Serial connection to the ESP32
        # Directly initialize the serial connection
        self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

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

        self.lights_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.lights_sock.bind((HOST_IP, LIGHTS_PORT))
        self.lights_sock.listen(1)

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

        self.lights_thread = threading.Thread(target=self.lights_control_thread, daemon=True)
        self.lights_thread.start()

        # Start the serial comms thread
        self.start_serial_thread()

        print("Robot server has started")

    def send_serial_command(self, command):
        """Send JSON command via serial to the ESP32."""
        if self.serial_conn.is_open:
            # Convert the command, which is a dictionary, to a string
            command_str = json.dumps(command)
            #print(f"Sending command to ESP32: {command_str}")
            # Convert the string to binary
            self.serial_conn.write(command_str.encode() + b'\n')
            self.serial_conn.flush()  # Ensure data is written immediately
        else:
            print("Serial connection is not open.")

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
                    self.enqueue_command(command, priority=1)
                except json.JSONDecodeError:
                    print('Motor control thread - invalid JSON received')
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
                    self.enqueue_command(command, priority=2)
                except json.JSONDecodeError:
                    print('Gimbal control thread - invalid JSON received')
            conn.close()

    def lights_control_thread(self):
        """Thread to handle incoming commands (not just motors) from the laptop via sockets."""
        while True:
            conn, _ = self.lights_sock.accept()
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                try:
                    command = json.loads(data.decode('utf-8'))
                    self.enqueue_command(command, priority=2)
                except json.JSONDecodeError:
                    print('Lights control thread - invalid JSON received')
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
                    #self.send_serial_command(command)
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

    def enqueue_command(self, command, priority):
        """Add a command to the priority queue."""
        #self.command_queue.put((priority, command))
        self.command_queue.put((priority, time.time(), command))

    def process_serial_queue(self):
        """Thread to process commands from the queue."""
        while True:
            priority, timestamp, command = self.command_queue.get()
            try:
                with self.serial_lock:
                    #command_str = json.dumps(command)
                    command_str = command
                    self.send_serial_command(command_str)
                    #print(f'Sent command: {command} with priority {priority}')
            finally:
                # Only mark task as done if the command was processed without interruption
                self.command_queue.task_done()  # Mark this task as done

    def start_serial_thread(self):
        """Start the serial processing thread."""
        serial_thread = threading.Thread(target=self.process_serial_queue, daemon=True)
        serial_thread.start()

    def close_serial_connection(self):
        if self.serial_conn.is_open:
            self.serial_conn.close()
            print("Serial connection closed.")

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
        rover.close_serial_connection()