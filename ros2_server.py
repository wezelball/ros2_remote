import socket
import json
import time
import serial
import threading

from base_ctrl import BaseController

# Connect to the ESP32 over USB (adjust port and baudrate)
#esp32_serial = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
base = BaseController('/dev/ttyAMA0', 115200)

def send_to_esp32(command):
    base.send_command(command)
    #print(f'Sent to ESP32: {command}')

def receive_feedback_from_esp32():
    # Implement an infinite loop to continuously monitor serial port data.
    while True:
        try:
            # Read a line of data from the serial port, decode it into a 'utf-8' formatted string, and attempt to convert it into a JSON object.
            data_recv_buffer = json.loads(base.rl.readline().decode('utf-8'))
            # Check if the parsed data contains the key 'T'.
            if 'T' in data_recv_buffer:
                # If the value of 'T' is 1001, print the received data and break out of the loop.
                if data_recv_buffer['T'] == 1001:
                    #print(data_recv_buffer)
                    return data_recv_buffer
                    break
        # If an exception occurs while reading or processing the data, ignore the exception and continue to listen for the next line of data.
        except:
            print('receive feedback from esp32 failed')
            return None


def disable_auto_feedback():
    """Send command to disable automatic feedback from ESP32."""
    disable_command = {"T": 131}
    send_to_esp32(disable_command)

def request_feedback():
    """Request feedback from the ESP32."""
    feedback_command = {"T": 130}
    send_to_esp32(feedback_command)

def motor_control_server():
    """Handle motor control requests from the laptop."""
    host = '0.0.0.0'
    port = 5000  # Different port for motor control

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f'Motor control server listening on {host}:{port}')

        while True:
            conn, addr = s.accept()
            with conn:
                print(f'Connected by {addr}')
                data = conn.recv(1024)

                if data:
                    print(f'data: {data}')
                    motor_command = json.loads(data.decode('utf-8'))
                    # Send the motor command to the ESP32
                    send_to_esp32(motor_command)
                    conn.sendall(b'OK')  # Acknowledge receipt of the command
                else:
                    print('no data')

def feedback_server():
    """Start the server to handle feedback requests from the laptop."""
    host = '0.0.0.0'
    port = 6000  # Port for feedback requests

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f'Feedback server listening on {host}:{port}')

        while True:
            conn, addr = s.accept()
            with conn:
                print(f'Connected by {addr}')
                data = conn.recv(1024)

                #print(f'data received from client {data}')

                if data:
                    request = json.loads(data.decode('utf-8'))

                    # The feedback command was given
                    if request.get('T') == 130:
                        # Get the feedback and send it back
                        feedback = receive_feedback_from_esp32()

                        if feedback:
                            # Extract odometer and battery voltage from feedback
                            response = {
                                "odl": feedback.get('odl', 0),
                                "odr": feedback.get('odr', 0),
                                "battery_voltage": feedback.get('v', 0)
                            }
                            conn.sendall(json.dumps(response).encode('utf-8'))
                            print(f'Sent feedback response: {response}')

if __name__ == '__main__':
    # Disable automatic feedback from ESP32
    #disable_auto_feedback()

    # Run the motor control server and feedback server in separate threads
    motor_thread = threading.Thread(target=motor_control_server, daemon=True)
    feedback_thread = threading.Thread(target=feedback_server, daemon=True)

    motor_thread.start()
    feedback_thread.start()

    motor_thread.join()
    feedback_thread.join()
