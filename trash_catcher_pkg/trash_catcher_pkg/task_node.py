import rclpy
from rclpy.node import Node
import math

from trash_catcher_pkg.arduino_driver import ArduinoDriver

class MotorTaskNode(Node):
    def __init__(self):
        super().__init__('motor_task_node')
        
        # Connection configured for SSH socket tunnel[cite: 1]
        self.port = 'socket://localhost:9999' #[cite: 1]
        
        # Stepper kinematics configuration[cite: 1]
        self.wheel_diameter_mm = 65.0 #[cite: 1]
        self.steps_per_rev = 1600.0 #[cite: 1]
        self.steps_per_mm = self.steps_per_rev / (math.pi * self.wheel_diameter_mm) #[cite: 1]
        
        # Speed and Control tuning
        self.kp_pos = 10.0           
        self.max_velocity = 4000.0   
        self.max_acceleration = 8000.0 # Steps per second squared
        self.current_velocity = 0.0  # Tracks actual speed for ramping
        self.tolerance = 5 #[cite: 1]
        
        self.dt = 0.02 # 50Hz control loop
        
        self.driver = ArduinoDriver(port=self.port) #[cite: 1]
        if not self.driver.connect(): #[cite: 1]
            self.get_logger().error("Shutting down node due to TCP socket connection failure.") #[cite: 1]
            return #[cite: 1]

        self.get_logger().info("Connected to local bridge socket. Starting Task Sequence.") #[cite: 1]

        self.state = 'INIT' #[cite: 1]
        self.target_steps = 0 #[cite: 1]
        self.wait_start_time = 0.0 #[cite: 1]
        
        self.timer = self.create_timer(self.dt, self.control_loop) #[cite: 1]

    def control_loop(self):
        current_steps = self.driver.get_latest_ticks() #[cite: 1]
        
        if current_steps is None: #[cite: 1]
            return  #[cite: 1]

        # STATE: INITIALIZE[cite: 1]
        if self.state == 'INIT': #[cite: 1]
            self.get_logger().info("Moving forward 100mm...") #[cite: 1]
            distance_mm = 500.0 #[cite: 1]
            self.target_steps = current_steps + (distance_mm * self.steps_per_mm) #[cite: 1]
            self.state = 'MOVING_FORWARD' #[cite: 1]

        # STATE: MOVING FORWARD[cite: 1]
        elif self.state == 'MOVING_FORWARD': #[cite: 1]
            if self.drive_to_target(current_steps, self.target_steps): #[cite: 1]
                self.get_logger().info("Arrived. Waiting 2 seconds...") #[cite: 1]
                self.current_velocity = 0.0 
                self.driver.send_velocity(0.0) #[cite: 1]
                self.wait_start_time = self.get_clock().now().nanoseconds / 1e9 #[cite: 1]
                self.state = 'WAITING' #[cite: 1]

        # STATE: WAITING[cite: 1]
        elif self.state == 'WAITING': #[cite: 1]
            current_time = self.get_clock().now().nanoseconds / 1e9 #[cite: 1]
            if (current_time - self.wait_start_time) >= 2.0: #[cite: 1]
                self.get_logger().info("Wait complete. Moving backward 100mm...") #[cite: 1]
                distance_mm = -100.0 #[cite: 1]
                self.target_steps = current_steps + (distance_mm * self.steps_per_mm) #[cite: 1]
                self.state = 'MOVING_BACKWARD' #[cite: 1]

        # STATE: MOVING BACKWARD[cite: 1]
        elif self.state == 'MOVING_BACKWARD': #[cite: 1]
            if self.drive_to_target(current_steps, self.target_steps): #[cite: 1]
                self.get_logger().info("Sequence complete.") #[cite: 1]
                self.current_velocity = 0.0
                self.driver.send_velocity(0.0) #[cite: 1]
                self.state = 'DONE' #[cite: 1]

        # STATE: DONE[cite: 1]
        elif self.state == 'DONE': #[cite: 1]
            pass  #[cite: 1]

    def drive_to_target(self, current_steps, target_steps):
        error = target_steps - current_steps #[cite: 1]
        
        if abs(error) <= self.tolerance: #[cite: 1]
            return True  #[cite: 1]
            
        target_vel = self.kp_pos * error
        
        # Apply velocity limits
        if target_vel > self.max_velocity: target_vel = self.max_velocity
        if target_vel < -self.max_velocity: target_vel = -self.max_velocity
            
        # Apply acceleration limits (Slew Rate Limiting)
        vel_error = target_vel - self.current_velocity
        max_vel_step = self.max_acceleration * self.dt
        
        if vel_error > max_vel_step:
            self.current_velocity += max_vel_step
        elif vel_error < -max_vel_step:
            self.current_velocity -= max_vel_step
        else:
            self.current_velocity = target_vel
            
        self.driver.send_velocity(self.current_velocity)
        return False

    def destroy_node(self): #[cite: 1]
        self.driver.close() #[cite: 1]
        super().destroy_node() #[cite: 1]

def main(args=None): #[cite: 1]
    rclpy.init(args=args) #[cite: 1]
    node = MotorTaskNode() #[cite: 1]
    try: #[cite: 1]
        rclpy.spin(node) #[cite: 1]
    except KeyboardInterrupt: #[cite: 1]
        pass #[cite: 1]
    finally: #[cite: 1]
        node.destroy_node() #[cite: 1]
        rclpy.shutdown() #[cite: 1]

if __name__ == '__main__': #[cite: 1]
    main() #[cite: 1]