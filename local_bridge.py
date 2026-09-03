import serial
import socket
import time

COM_PORT = 'COM3'  # Update to your local laptop COM port
BAUD_RATE = 115200
TCP_PORT = 9999

# 1. Connect to local USB Arduino
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.01)
time.sleep(2)
print(f"Connected to Arduino on {COM_PORT}")

# 2. Setup TCP Server for SSH Tunnel
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('0.0.0.0', TCP_PORT))
server_socket.listen(1)
print(f"Bridge active. Listening on localhost:{TCP_PORT}...")

conn, addr = server_socket.accept()
print(f"Connected to cluster container!")
conn.setblocking(False)

# 3. Two-way relay (COM Port <-> TCP Socket)
try:
    while True:
        # Serial -> Socket
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            conn.sendall(data)

        # Socket -> Serial
        try:
            data = conn.recv(1024)
            if data:
                ser.write(data)
        except (BlockingIOError, socket.error):
            pass
except KeyboardInterrupt:
    print("\nClosing bridge.")
finally:
    conn.close()
    ser.close()