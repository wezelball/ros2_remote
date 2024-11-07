import struct
import socket
import serial
import json
import time
import math
from queue import PriorityQueue
import threading
import cv2
from picamera2 import Picamera2
import numpy as np


HOST_IP = '0.0.0.0'
MOTOR_PORT = 5000
LIGHTS_PORT = 5500
FEEDBACK_PORT = 6000
GIMBAL_PORT = 7000
VIDEO_PORT = 8000
SERIAL_PORT = '/dev/ttyAMA0'
LIDAR_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
LIDAR_BAUD_RATE = 230400
PACKET_LENGTH = 47
MAX_QUEUE_ITEMS = 10
COMMAND_EXPIRY_MS = 50

class RoverController():
    def __init__(self):
        # Initialize priority queue and lock for serial access
        self.command_queue = PriorityQueue(maxsize=MAX_QUEUE_ITEMS)
        self.serial_lock = threading.Lock()

        # Serial connection to the ESP32
        # Directly initialize the serial connection
        self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

        # Serial connection to the LIDAR
        self.lidar_conn = serial.Serial(LIDAR_PORT, LIDAR_BAUD_RATE, timeout=1)

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

        self.lidar_data = b''  # Initialize to store latest LiDAR data as bytes

        # Start all of the threads
        self.start_threads()

        print("Robot server has started")

    def start_threads(self):
        """Initialize and start all component threads."""
        threading.Thread(target=self.motor_control_thread, daemon=True).start()
        threading.Thread(target=self.feedback_thread_func, daemon=True).start()
        threading.Thread(target=self.gimbal_control_thread, daemon=True).start()
        threading.Thread(target=self.video_stream_thread, daemon=True).start()
        threading.Thread(target=self.lights_control_thread, daemon=True).start()
        threading.Thread(target=self.process_serial_queue, daemon=True).start()
        threading.Thread(target=self.start_lidar_server, daemon=True).start()
        threading.Thread(target=self.lidar_read_loop, daemon=True).start()

    def start_lidar_server(self):
        """Function to start a dedicated LiDAR server on port 9000."""
        self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.lidar_socket.bind(('0.0.0.0', 9000))
        self.lidar_socket.listen()

        print("LiDAR server listening on port 9000")

        while True:
            client_conn, client_addr = self.lidar_socket.accept()
            print(f"LiDAR client connected from {client_addr}")

            lidar_thread = threading.Thread(target=self.handle_lidar_client, args=(client_conn,), daemon=True)
            lidar_thread.start()

    def handle_lidar_client(self, client_conn):
        """Send LiDAR data continuously to the connected client."""
        try:
            while True:
                lidar_data = self.get_latest_lidar_data()
                if lidar_data:
                    # Send LiDAR data to the client in a structured format
                    client_conn.sendall(json.dumps(lidar_data).encode())
                    #print(f'lidar data: {lidar_data}')
                time.sleep(0.1)  # Adjust for your desired data frequency
        except Exception as e:
            print(f"Error in LiDAR client handler: {e}")
        finally:
            client_conn.close()

    def get_latest_lidar_data(self):
        """Fetch the latest LiDAR data as a bytes object.
        This function should return the LiDAR data packet formatted as needed.
        """
        # For example, suppose you have a variable `self.lidar_data` that stores the latest data packet
        if hasattr(self, 'lidar_data'):
            return self.lidar_data
        else:
            return None

    def lidar_read_loop(self):
        """Continuously read data from the LiDAR device and update the latest packet."""
        while True:
            try:
                #print('DEBUG: before self.read_lidar_packet()')
                packet = self.read_lidar_packet()
                if packet:
                    #print(f"LiDAR Packet: {packet}")
                    self.lidar_data = self.process_lidar_data(packet)
                    #print(f'self.lidar_data: {self.lidar_data}')
            except Exception as e:
                print(f"LiDAR read error: {e}")
            time.sleep(0.01)  # Adjust for LiDAR read frequency

    def read_lidar_packet(self):
        """Read and parse a single LiDAR data packet."""
        # Find the start character (0x54) in the data stream
        while True:
            byte = self.lidar_conn.read(1)
            if len(byte) == 1 and byte[0] == 0x54:
                break

        # Read the rest of the packet
        data = self.lidar_conn.read(PACKET_LENGTH - 1)
        if len(data) != PACKET_LENGTH - 1:
            print("Incomplete packet received")
            return None



        # Parse packet contents and return data points
        data_length = data[0]
        num_points = data_length & 0x1F
        radar_speed = struct.unpack_from('<H', data, 1)[0] / 100.0
        start_angle_deg = struct.unpack_from('<H', data, 3)[0] / 100.0
        start_angle_rad = start_angle_deg * (math.pi / 180.0)
        end_angle_deg = struct.unpack_from('<H', data, 41)[0] / 100.0
        end_angle_rad = end_angle_deg * (math.pi / 180.0)


        # Calculate angular resolution
        angular_resolution = ((360.0 - start_angle_deg + end_angle_deg) if end_angle_deg < start_angle_deg
                              else (end_angle_deg - start_angle_deg)) / max((num_points - 1), 1)

        data_points = []
        # For each measurement, unpack the angle, distance, and confidence, and append
        # to data_points list
        for i in range(12):
            offset = 5 + i * 3
            distance = struct.unpack_from('<H', data, offset)[0]
            confidence = data[offset + 2]
            angle_deg = (start_angle_deg + i * angular_resolution) % 360
            angle_rad = angle_deg * (math.pi/180)
            data_point = [angle_rad, distance]
            data_points.append(data_point)

        #print (f'start_angle_deg: {start_angle_deg}')
        #print(f'end_angle_deg: {end_angle_deg}')
        #print(f'data_points: {data_points}')

        # Create actual LIDAR packet which includes min and max angles,
        # as well as each data point.
        lidar_packet = {
            "angle_min": start_angle_rad,  # minimum angle in the packet
            "angle_max": end_angle_rad,  # maximum angle in the packet
            # THIS IS THE PROBLEM I THINK
            "ranges": [[angle, distance] for angle, distance in data_points]
        }

        #print(f'DEBUG: lidar_packet: {lidar_packet}')
        return lidar_packet

    def process_lidar_data(self, lidar_packet):
        """Process LiDAR data points, e.g., for visualization or obstacle detection.
        data_points is a list of tuples where each tuple consists of angle, distance, and confidence.
        Here is an example:

        data_points: [(193.94, 2541, 176), (194.64636363636365, 2549, 177),...]

        equates to:

        Angle: 193.94, Distance: 2541 mm, Confidence: 176
        Angle: 194.65, Distance: 2549 mm, Confidence: 177
        ...

        There are 12 of these in the data_points packet.

        According to ChatGPT, process_lidar_data needs to return an array of tuples
        (angle, distance)

        Now I need to add scan angle_min and angle_max

        """

        return lidar_packet

    def send_serial_command(self, command):
        """Send JSON command via serial to the ESP32."""
        if self.serial_conn.is_open:
            # Convert the command, which is a dictionary, to a string
            command_str = json.dumps(command)
            print(f"Sending: {command_str}")
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
                # TODO: Sleep statement below for testing slowing down gimbal thread
                time.sleep(0.05)
                if not data:
                    break
                try:
                    command = json.loads(data.decode('utf-8'))
                    self.enqueue_command(command, priority=2)
                except json.JSONDecodeError:
                    pass
                    #print('Gimbal control thread - invalid JSON received')
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
        self.lights_sock.close()
        self.lidar_socket.close()

    def enqueue_command(self, command, priority):
        """Add a command to the priority queue with a priority and a timestamp."""
        self.command_queue.put((priority, time.time(), command))

    def remove_expired_commands(self):
        """Remove commands from the queue that have expired"""
        current_time_ = time.time()
        temp_queue = PriorityQueue(maxsize=MAX_QUEUE_ITEMS)

        while not self.command_queue.empty():
            priority, timestamp, command = self.command_queue.get()
            # check if the command is still valid
            if (current_time_ - timestamp) * 1000 <= COMMAND_EXPIRY_MS:
                # If not too old, reinsert into temp queue
                temp_queue.put((priority, timestamp, command))
            else:
                print("Found expired command")

        # Replace the command_queue with only fresh non-expired commands
        self.command_queue = temp_queue

    def process_serial_queue(self):
        """Thread to process commands from the queue."""
        while True:
            # Remove commands older than COMMAND_EXPIRY_MS
            #self.remove_expired_commands()
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

    def close_serial_connection(self):
        if self.serial_conn.is_open:
            self.serial_conn.close()
            print("ESP32 serial connection closed.")
        if self.lidar_conn.is_open:
            self.lidar_conn.close()
            print("LIDAR serial connection closed.")

# Example usage
if __name__ == '__main__':
    rover = RoverController()
    rover.disable_auto_feedback()

    try:
        while True:
            time.sleep(1)
            # Place logging/status checks here
    except KeyboardInterrupt:
        print("Shutting down Robot server")
        rover.close_sockets()
        rover.close_serial_connection()
        print("Robot server has stopped.")