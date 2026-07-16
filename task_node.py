import rclpy
from rclpy.node import Node
import math

# Import the driver we created in the other file
from arduino_driver import ArduinoDriver

class MotorTaskNode(Node):
    def __init__(self):
        super().__init__('motor_task_node')
        
        # --- Configuration ---
        self.port = 'COM3' # Update as needed (/dev/ttyUSB0 for Linux)
        self.wheel_diameter_mm = 65.0
        self.ticks_per_rev = 341.0
        self.ticks_per_mm = self.ticks_per_rev / (math.pi * self.wheel_diameter_mm)
        
        self.kp_pos = 2.0
        self.max_velocity = 600.0
        self.tolerance = 10
        
        # --- Initialize Hardware Driver ---
        self.driver = ArduinoDriver(port=self.port)
        if not self.driver.connect():
            self.get_logger().error("Shutting down node due to connection failure.")
            return

        self.get_logger().info("Connected to Arduino. Starting Task Sequence.")

        # --- State Machine Variables ---
        self.state = 'INIT'
        self.target_ticks = 0
        self.wait_start_time = 0.0
        
        # Create a timer to run the control loop at 50Hz (0.02 seconds)
        self.timer = self.create_timer(0.02, self.control_loop)

    def control_loop(self):
        """This function runs 50 times a second. No while loops allowed!"""
        current_ticks = self.driver.get_latest_ticks()
        
        if current_ticks is None:
            return # Wait for valid telemetry

        # --- STATE: INITIALIZE ---
        if self.state == 'INIT':
            self.get_logger().info("Moving forward 100mm...")
            distance_mm = 100.0
            self.target_ticks = current_ticks + (distance_mm * self.ticks_per_mm)
            self.state = 'MOVING_FORWARD'

        # --- STATE: MOVING FORWARD ---
        elif self.state == 'MOVING_FORWARD':
            if self.drive_to_target(current_ticks, self.target_ticks):
                self.get_logger().info("Arrived. Waiting 2 seconds...")
                self.driver.send_velocity(0.0)
                # Record the current ROS time in seconds
                self.wait_start_time = self.get_clock().now().nanoseconds / 1e9 
                self.state = 'WAITING'

        # --- STATE: WAITING ---
        elif self.state == 'WAITING':
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - self.wait_start_time) >= 2.0:
                self.get_logger().info("Wait complete. Moving backward 100mm...")
                distance_mm = -100.0
                self.target_ticks = current_ticks + (distance_mm * self.ticks_per_mm)
                self.state = 'MOVING_BACKWARD'

        # --- STATE: MOVING BACKWARD ---
        elif self.state == 'MOVING_BACKWARD':
            if self.drive_to_target(current_ticks, self.target_ticks):
                self.get_logger().info("Sequence complete.")
                self.driver.send_velocity(0.0)
                self.state = 'DONE'

        # --- STATE: DONE ---
        elif self.state == 'DONE':
            pass # Keep node alive, but do nothing. Or trigger rclpy.shutdown()

    def drive_to_target(self, current_ticks, target_ticks):
        """Calculates P-control velocity. Returns True if arrived, False otherwise."""
        error = target_ticks - current_ticks
        
        if abs(error) <= self.tolerance:
            return True # We arrived
            
        cmd_vel = self.kp_pos * error
        
        # Clamp velocity
        if cmd_vel > self.max_velocity: cmd_vel = self.max_velocity
        if cmd_vel < -self.max_velocity: cmd_vel = -self.max_velocity
            
        self.driver.send_velocity(cmd_vel)
        return False

    def destroy_node(self):
        """Ensure the serial port is closed when the node is killed."""
        self.driver.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()