import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2

class MotionTracker(Node):
    def __init__(self):
        super().__init__('motion_tracker')
        
        # Subscribe to the camera feed
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # Publish the (X, Y) coordinate of the moving object
        self.publisher_ = self.create_publisher(Point, '/target_position', 10)
        self.bridge = CvBridge()
        
        # Initialize OpenCV Background Subtractor for motion detection
        self.back_sub = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=False)
        self.get_logger().info("Vision Tracker started. Waiting for video stream...")

    def image_callback(self, msg):
        try:
            # Convert ROS Image back to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Apply the background subtractor to find motion
        fg_mask = self.back_sub.apply(frame)

        # Clean up the mask using morphological operations (removes noise)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours (outlines of moving objects)
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Assume the largest moving object is our target
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Filter out tiny movements (noise)
            if cv2.contourArea(largest_contour) > 500:
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Calculate the center point of the object
                center_x = float(x + w / 2.0)
                center_y = float(y + h / 2.0)

                # Publish the center coordinates
                point_msg = Point()
                point_msg.x = center_x
                point_msg.y = center_y
                point_msg.z = 0.0  # Unused, but available if you calculate depth/area later
                
                self.publisher_.publish(point_msg)
                self.get_logger().info(f"Target at X: {center_x:.1f}, Y: {center_y:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = MotionTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()