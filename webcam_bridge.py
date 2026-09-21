import cv2
import socket
import struct

TCP_PORT = 9998  # Use a different port than your Arduino bridge

def start_video_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', TCP_PORT))
    server_socket.listen(1)
    print(f"Video Bridge active. Listening on localhost:{TCP_PORT}...")

    conn, addr = server_socket.accept()
    print("Connected to cluster!")

    # Open local webcam
    cap = cv2.VideoCapture(1)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
                
            # Compress frame to JPEG to save bandwidth over the SSH tunnel
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            result, buffer = cv2.imencode('.jpg', frame, encode_param)
            data = buffer.tobytes()

            # Pack the size of the data as a 4-byte unsigned long (<L)
            # This ensures the receiver knows exactly how many bytes to read
            header = struct.pack("<L", len(data))
            
            # Send header then frame data
            conn.sendall(header + data)
            
    except KeyboardInterrupt:
        print("\nClosing video bridge.")
    except Exception as e:
        print(f"Connection lost: {e}")
    finally:
        cap.release()
        conn.close()
        server_socket.close()

if __name__ == '__main__':
    start_video_server()