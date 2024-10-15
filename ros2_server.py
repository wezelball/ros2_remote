import socket
import json
import time
import serial
import threading

from base_ctrl import BaseController

# Connect to the ESP32 over USB (adjust port and baudrate)
#esp32_serial = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
base = BaseController('/dev/ttyAMA0', 115200)

"""
def send_to_esp32(command):
    #Send a command to the ESP32 over USB
    try:
        esp32_serial.write(json.dumps(command).encode('utf-8'))
        print(f'Sent to ESP32: {command}')
    except Exception as e:
        print(f'Error sending to ESP32: {str(e)}')
"""

def send_to_esp32(command):
    base.send_command(command)
    print(f'Sent to ESP32: {command}')

def receive_feedback_from_esp32():
    """Read feedback data from the ESP32."""
    try:
        feedback_data = base.feedback_data()
        #feedback_data = esp32_serial.readline().decode('utf-8')
        if feedback_data:
            print(f'Received feedback from ESP32: {feedback_data}')
            feedback = feedback_data
            #feedback = json.loads(feedback_data)
            #print(f'Received feedback from ESP32: {feedback}')
            return feedback
    except Exception as e:
        print(f'Error receiving feedback: {str(e)}')
    return None

def disable_auto_feedback():
    """Send command to disable automatic feedback from ESP32."""
    disable_command = {"T": 131, "cmd": 0}
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

                if data:
                    request = json.loads(data.decode('utf-8'))

                    if request.get('command') == 'request_feedback':
                        # Request feedback from ESP32
                        request_feedback()

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
    disable_auto_feedback()

    # Run the motor control server and feedback server in separate threads
    motor_thread = threading.Thread(target=motor_control_server, daemon=True)
    feedback_thread = threading.Thread(target=feedback_server, daemon=True)

    motor_thread.start()
    feedback_thread.start()

    motor_thread.join()
    feedback_thread.join()
