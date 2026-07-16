import serial
import time

class ArduinoDriver:
    def __init__(self, port='COM3', baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None

    def connect(self):
        """Establishes the serial connection."""
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2) # Wait for Arduino to reset
            self.serial_conn.reset_input_buffer()
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to Arduino on {self.port}: {e}")
            return False

    def send_velocity(self, target_vel):
        """Sends a velocity command (ticks/sec) to the Arduino."""
        if self.serial_conn and self.serial_conn.is_open:
            cmd = f"V {float(target_vel):.2f}\n"
            self.serial_conn.write(cmd.encode('utf-8'))

    def get_latest_ticks(self):
        """Reads the serial buffer to get the absolute newest tick count."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None

        ticks = None
        # Drain the buffer to catch up to real-time
        while self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                parts = line.split(',')
                if len(parts) >= 3:
                    ticks = int(parts[0])
            except Exception:
                pass # Ignore garbled data
                
        return ticks
        
    def close(self):
        """Safely closes the connection."""
        if self.serial_conn:
            self.send_velocity(0.0) # Stop motor before closing
            self.serial_conn.close()