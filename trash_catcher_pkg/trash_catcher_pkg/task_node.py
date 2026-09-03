import rclpy
from rclpy.node import Node
import math

from trash_catcher_pkg.arduino_driver import ArduinoDriver

class MotorTaskNode(Node):
    def __init__(self):
        super().__init__('motor_task_node')
        
        # Connection configured for SSH socket tunnel
        self.port = 'socket://localhost:9999'
        
        # Stepper kinematics configuration
        self.wheel_diameter_mm = 65.0
        self.steps_per_rev = 1600.0  # 200 steps/rev * 8 microsteps
        self.steps_per_mm = self.steps_per_rev / (math.pi * self.wheel_diameter_mm)
        
        self.kp_pos = 1.0 
        self.max_velocity = 800.0  # steps per second
        self.tolerance = 5  # steps
        
        self.driver = ArduinoDriver(port=self.port)
        if not self.driver.connect():
            self.get_logger().error("Shutting down node due to TCP socket connection failure.")
            return

        self.get_logger().info("Connected to local bridge socket. Starting Task Sequence.")

        self.state = 'INIT'
        self.target_steps = 0
        self.wait_start_time = 0.0
        
        self.timer = self.create_timer(0.02, self.control_loop)

    def control_loop(self):
        current_steps = self.driver.get_latest_ticks()
        
        if current_steps is None:
            return 

        # STATE: INITIALIZE
        if self.state == 'INIT':
            self.get_logger().info("Moving forward 100mm...")
            distance_mm = 100.0
            self.target_steps = current_steps + (distance_mm * self.steps_per_mm)
            self.state = 'MOVING_FORWARD'

        # STATE: MOVING FORWARD
        elif self.state == 'MOVING_FORWARD':
            if self.drive_to_target(current_steps, self.target_steps):
                self.get_logger().info("Arrived. Waiting 2 seconds...")
                self.driver.send_velocity(0.0)
                self.wait_start_time = self.get_clock().now().nanoseconds / 1e9
                self.state = 'WAITING'

        # STATE: WAITING
        elif self.state == 'WAITING':
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - self.wait_start_time) >= 2.0:
                self.get_logger().info("Wait complete. Moving backward 100mm...")
                distance_mm = -100.0
                self.target_steps = current_steps + (distance_mm * self.steps_per_mm)
                self.state = 'MOVING_BACKWARD'

        # STATE: MOVING BACKWARD
        elif self.state == 'MOVING_BACKWARD':
            if self.drive_to_target(current_steps, self.target_steps):
                self.get_logger().info("Sequence complete.")
                self.driver.send_velocity(0.0)
                self.state = 'DONE'

        # STATE: DONE
        elif self.state == 'DONE':
            pass 

    def drive_to_target(self, current_steps, target_steps):
        error = target_steps - current_steps
        
        if abs(error) <= self.tolerance:
            return True 
            
        cmd_vel = self.kp_pos * error
        
        if cmd_vel > self.max_velocity: cmd_vel = self.max_velocity
        if cmd_vel < -self.max_velocity: cmd_vel = -self.max_velocity
            
        self.driver.send_velocity(cmd_vel)
        return False

    def destroy_node(self):
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