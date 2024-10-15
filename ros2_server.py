import socket
import json
import serial
from base_ctrl import BaseController

# Connect to the ESP32 over USB (adjust port and baudrate)
#esp32_serial = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
base = BaseController('/dev/ttyAMA0', 115200)

def send_to_esp32(command):
    try:
        # Convert the command to a JSON string and send it to the ESP32
        esp32_serial.write(json.dumps(command).encode('utf-8'))
        print(f'Sent to ESP32: {command}')
    except Exception as e:
        print(f'Error sending to ESP32: {str(e)}')

def start_server():
    host = '0.0.0.0'  # Listen on all network interfaces
    port = 5000       # Same port as the ROS2 node on the laptop

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f'Server listening on {host}:{port}')

        while True:
            conn, addr = s.accept()
            with conn:
                print(f'Connected by {addr}')
                data = conn.recv(1024)  # Receive data from the laptop

                if data:
                    # Decode the JSON data and send it to the ESP32
                    command = json.loads(data.decode('utf-8'))
                    #send_to_esp32(command)
                    base.send_command(command)
                    print(f'Sent to ESP32: {command}')

if __name__ == '__main__':
    start_server()
