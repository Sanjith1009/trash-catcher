import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import struct
import numpy as np

class TcpVideoPublisher(Node):
    def __init__(self):
        super().__init__('tcp_video_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Connect to the local laptop via SSH tunnel
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.get_logger().info("Connecting to local video bridge on localhost:9998...")
        self.client_socket.connect(('127.0.0.1', 9998))
        self.get_logger().info("Connected! Receiving video stream...")

        # Run the receiving loop fast
        self.timer = self.create_timer(0.01, self.receive_frame)
        self.payload_size = struct.calcsize("<L")
        self.data_buffer = b""

    def recvall(self, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = bytearray()
        while len(data) < n:
            packet = self.client_socket.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data

    def receive_frame(self):
        try:
            # 1. Receive the 4-byte header containing the frame size
            header = self.recvall(self.payload_size)
            if not header:
                return
                
            msg_size = struct.unpack("<L", header)[0]
            
            # 2. Receive the actual compressed frame data
            frame_data = self.recvall(msg_size)
            if not frame_data:
                return

            # 3. Decode JPEG bytes back into an OpenCV image
            frame_np = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)

            # 4. Convert and publish to ROS 2
            if frame is not None:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = 'camera_frame'
                self.publisher_.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Error receiving frame: {e}")

    def destroy_node(self):
        self.client_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TcpVideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()