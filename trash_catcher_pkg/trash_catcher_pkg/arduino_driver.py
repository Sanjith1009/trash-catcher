import serial
import time

class ArduinoDriver:
    def __init__(self, port='socket://localhost:9999', baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None

    def connect(self):
        """Establishes the serial connection or TCP socket bridge."""
        try:
            if self.port.startswith('socket://'):
                self.serial_conn = serial.serial_for_url(self.port, timeout=1)
            else:
                self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
                
            time.sleep(2)
            return True
        except Exception as e:
            print(f"Failed to connect on {self.port}: {e}")
            return False

    def send_velocity(self, target_vel):
        """Sends a velocity command (steps/sec) to the Arduino."""
        if self.serial_conn and self.serial_conn.is_open:
            cmd = f"V {float(target_vel):.2f}\n"
            self.serial_conn.write(cmd.encode('utf-8'))

    def get_latest_ticks(self):
        """Reads the buffer to get the absolute newest step count."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None

        ticks = None
        while self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                parts = line.split(',')
                if len(parts) >= 3:
                    ticks = int(parts[0])
            except Exception:
                pass 
                
        return ticks
        
    def close(self):
        """Safely closes the connection."""
        if self.serial_conn:
            self.send_velocity(0.0)
            self.serial_conn.close()