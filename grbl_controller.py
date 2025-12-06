#!/usr/bin/env python3
"""
GRBL CNC Controller
A script to communicate with GRBL-based CNC machines over serial
"""

import serial
import time
import sys
import argparse
from typing import Optional


class GRBLController:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize GRBL controller
        
        Args:
            port: Serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0)
            baudrate: Communication speed (default 115200 for GRBL)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None
        
    def connect(self) -> bool:
        """Connect to the CNC machine"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for GRBL to initialize
            
            # Clear the welcome message
            self.serial_conn.flushInput()
            
            print(f"✓ Connected to {self.port} at {self.baudrate} baud")
            
            # Get GRBL version
            response = self.send_command("$I")
            if response:
                print(f"Machine info: {response}")
            
            return True
            
        except serial.SerialException as e:
            print(f"✗ Error connecting to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the CNC machine"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Disconnected from CNC machine")
    
    def send_command(self, command: str, wait_for_ok: bool = True) -> str:
        """
        Send a command to GRBL and return the response
        
        Args:
            command: GRBL command to send
            wait_for_ok: Wait for 'ok' or 'error' response
            
        Returns:
            Response from GRBL
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("✗ Not connected to machine")
            return ""
        
        try:
            # Strip whitespace and add newline
            command = command.strip() + '\n'
            
            # Send command
            self.serial_conn.write(command.encode('utf-8'))
            print(f">> {command.strip()}")
            
            # Read response
            responses = []
            if wait_for_ok:
                while True:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        print(f"<< {line}")
                        responses.append(line)
                        
                        # Stop reading after 'ok' or 'error'
                        if line.startswith('ok') or line.startswith('error'):
                            break
            else:
                # Just read available data
                time.sleep(0.1)
                while self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        print(f"<< {line}")
                        responses.append(line)
            
            return '\n'.join(responses)
            
        except Exception as e:
            print(f"✗ Error sending command: {e}")
            return ""
    
    def get_status(self) -> str:
        """Get real-time status from GRBL"""
        return self.send_command("?", wait_for_ok=False)
    
    def soft_reset(self):
        """Send soft reset command (Ctrl-X)"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(b'\x18')
            print(">> Soft reset sent")
            time.sleep(2)
            # Read any responses
            while self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line:
                    print(f"<< {line}")
    
    def feed_hold(self):
        """Pause the current operation"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(b'!')
            print(">> Feed hold sent")
    
    def cycle_resume(self):
        """Resume from feed hold"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(b'~')
            print(">> Cycle resume sent")
    
    def interactive_mode(self):
        """Run interactive mode for sending commands"""
        print("\n" + "="*60)
        print("GRBL Interactive Mode")
        print("="*60)
        print("Commands:")
        print("  Type GRBL commands directly (e.g., G0 X10 Y20)")
        print("  ? - Get status")
        print("  $$ - View settings")
        print("  $H - Home machine")
        print("  reset - Soft reset (Ctrl-X)")
        print("  hold - Feed hold (!)")
        print("  resume - Cycle resume (~)")
        print("  quit - Exit")
        print("="*60 + "\n")
        
        while True:
            try:
                cmd = input("GRBL> ").strip()
                
                if not cmd:
                    continue
                
                if cmd.lower() in ['quit', 'exit', 'q']:
                    break
                elif cmd.lower() == 'reset':
                    self.soft_reset()
                elif cmd.lower() == 'hold':
                    self.feed_hold()
                elif cmd.lower() == 'resume':
                    self.cycle_resume()
                else:
                    self.send_command(cmd)
                
                print()  # Empty line for readability
                
            except KeyboardInterrupt:
                print("\n\nInterrupted. Exiting...")
                break
            except Exception as e:
                print(f"Error: {e}")


def list_serial_ports():
    """List available serial ports"""
    import glob
    
    patterns = [
        '/dev/ttyUSB*',
        '/dev/ttyACM*',
        '/dev/ttyS*',
        '/dev/cu.usb*'  # macOS
    ]
    
    ports = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    
    return sorted(ports)


def main():
    parser = argparse.ArgumentParser(
        description='GRBL CNC Controller - Communicate with GRBL-based CNC machines',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s -p /dev/ttyUSB0                    # Interactive mode
  %(prog)s -p /dev/ttyUSB0 -c "G0 X10 Y20"    # Send single command
  %(prog)s -p /dev/ttyUSB0 -c "$$"            # View GRBL settings
  %(prog)s --list                             # List available ports
        """
    )
    
    parser.add_argument('-p', '--port', type=str, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('-c', '--command', type=str, help='Send single command and exit')
    parser.add_argument('--list', action='store_true', help='List available serial ports')
    
    args = parser.parse_args()
    
    # List ports if requested
    if args.list:
        print("Available serial ports:")
        ports = list_serial_ports()
        if ports:
            for port in ports:
                print(f"  {port}")
        else:
            print("  No serial ports found")
        return
    
    # Check if port is specified
    if not args.port:
        print("Error: Serial port not specified")
        print("\nAvailable ports:")
        ports = list_serial_ports()
        if ports:
            for port in ports:
                print(f"  {port}")
        else:
            print("  No serial ports found")
        print("\nUse -p to specify a port, e.g.: python3 grbl_controller.py -p /dev/ttyUSB0")
        return
    
    # Create controller and connect
    controller = GRBLController(args.port, args.baud)
    
    if not controller.connect():
        sys.exit(1)
    
    try:
        if args.command:
            # Single command mode
            controller.send_command(args.command)
        else:
            # Interactive mode
            controller.interactive_mode()
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()