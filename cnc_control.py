#!/usr/bin/env python3

import serial
import time
import re
import os
import glob
import simpleaudio as sa
from pynput import keyboard
import numpy as np
import platform
import cv2
import csv
import threading
from skimage.metrics import structural_similarity as ssim

import serial.tools.list_ports
import NetFT

class CNCController:
    def __init__(self, baudrate=115200, timeout=1):
        print("Initializing CNC Controller...")
        self.HOME_FILE = "home_position_aticenter.txt"  # File to store home position

        # Linux-specific serial port detection
        # On Linux, devices are typically at /dev/ttyUSB* or /dev/ttyACM*
        available_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")

        # Also check /dev/serial/by-id/ for more reliable device identification
        if os.path.exists("/dev/serial/by-id/"):
            by_id_ports = [os.path.join("/dev/serial/by-id/", p)
                          for p in os.listdir("/dev/serial/by-id/")]
            available_ports.extend(by_id_ports)

        # Filter out Arduino ports (we want GRBL/CNC)
        # Remove duplicates by converting to realpath
        available_ports = list(set([os.path.realpath(p) for p in available_ports if os.path.exists(p)]))

        if not available_ports:
            raise Exception("No serial ports found! Make sure your CNC is connected and you have permissions (add user to 'dialout' group)")

        print(f"Found {len(available_ports)} serial port(s): {available_ports}")

        # Try connecting to each available port
        for port in available_ports:
            try:
                print(f"Trying {port}...")
                self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
                time.sleep(2)  # Give GRBL time to initialize

                # Read and clear all startup messages from GRBL
                time.sleep(0.2)  # Give time for all startup messages
                startup_messages = []
                while self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        startup_messages.append(line)
                    time.sleep(0.05)

                # Send a status query to verify it's responding
                self.serial_port.write(b'?\n')
                time.sleep(0.2)
                response = self.serial_port.readline().decode('utf-8', errors='ignore').strip()

                print(f"Connected successfully to {port}")
                if startup_messages:
                    print(f"Startup messages: {', '.join(startup_messages)}")
                print(f"Status response: {response}")
                break  # Exit the loop if successful
            except (serial.SerialException, OSError) as e:
                print(f"Failed to connect to {port}: {e}")
                if hasattr(self, 'serial_port') and self.serial_port.is_open:
                    self.serial_port.close()

        else:
            # If no ports worked, raise an exception
            raise Exception("Could not connect to any available serial port. Check permissions with 'ls -l /dev/ttyUSB*' and add user to dialout group if needed: 'sudo usermod -a -G dialout $USER'")

    def wait_for_idle(self, timeout=100):
        """Wait for CNC to finish current movement"""
        # Small delay to allow CNC to start processing the command
        time.sleep(0.05)

        # Clear any old responses in the buffer
        self.serial_port.reset_input_buffer()

        # Now poll until idle
        start_time = time.time()
        while True:
            # Check timeout
            if time.time() - start_time > timeout:
                print(f"[WARNING] wait_for_idle timed out after {timeout}s")
                return False

            self.serial_port.write(b'?')
            time.sleep(0.05)  # Give time for response
            response = self.serial_port.readline().decode('utf-8').strip()
            print(f"Response: {response}")

            if response:  # Only process if we got a response
                # Print status for debugging
                print(f"[STATUS] {response}")

                # Check for alarm state
                if "Alarm" in response or "ALARM" in response or "ok" in response:
                    print("[ERROR] CNC is in ALARM state! Trying to Unlock.")
                    self.unlock_cnc()
                    time.sleep(4)
                    return True

                # Check if idle
                if response.startswith("<Idle") or "Idle" in response:
                    time.sleep(4)
                    return True

            time.sleep(0.1)  # Wait before next poll

    def unlock_cnc(self):
        """Unlock the CNC machine to allow manual control."""
        self.serial_port.write(b'$X\n')
        time.sleep(1)
        # command = f'$10=1\n'
        # self.serial_port.write(b"$10=0\n")
        # time.sleep(0.1)
        # self.serial_port.write(command.encode())
        print('CNC machine unlocked')

    def view_grbl_settings(self):
        """View the current Grbl settings."""
        self.serial_port.write(b'$$\n')
        time.sleep(0.5)
        response = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8').strip()
        print(f'Grbl settings:\n{response}')

    def reset_grbl_settings(self):
        """Reset Grbl settings to default values."""
        self.serial_port.write(b'$RST=$\n')
        time.sleep(0.5)
        response = self.serial_port.readline().decode('utf-8').strip()
        if 'ok' in response:
            print('Grbl settings reset to default')
        else:
            print('Failed to reset Grbl settings')

    def set_pulloff_distance(self, distance):
        """Set homing pull-off distance in mm for all axes."""
        self.serial_port.write(f'$27={distance}\n'.encode())
        time.sleep(0.5)
        response = self.serial_port.readline().decode('utf-8').strip()
        print(f'Pull-off distance set to {distance} mm')
        print(f'Response: {response}')

    def set_axis_pulloff_distance(self, axis, distance):
        """
        Set homing pull-off distance for a specific axis (GRBL 1.1+).
        axis: 'x', 'y', or 'z'
        distance: pulloff distance in mm
        """
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        if axis.lower() not in axis_map:
            print("Invalid axis. Use 'x', 'y', or 'z'")
            return

        axis_num = axis_map[axis.lower()]
        self.serial_port.write(f'$27.{axis_num}={distance}\n'.encode())
        time.sleep(0.5)
        response = self.serial_port.readline().decode('utf-8').strip()
        print(f'{axis.upper()}-axis pull-off distance set to {distance} mm')
        print(f'Response: {response}')

    def set_homing_direction(self, x_invert=False, y_invert=False, z_invert=False):
        """
        Set homing direction for each axis using $23 (homing dir invert mask).

        GRBL $23 setting uses a bitmask:
        - Bit 0 (value 1): Invert X homing direction
        - Bit 1 (value 2): Invert Y homing direction
        - Bit 2 (value 4): Invert Z homing direction

        Parameters:
            x_invert: True to invert X (home in positive direction), False for negative
            y_invert: True to invert Y (home in positive direction), False for negative
            z_invert: True to invert Z (home in positive direction), False for negative

        Example: To make X home positive, Y negative, Z negative:
            set_homing_direction(x_invert=True, y_invert=False, z_invert=False)
        """
        mask = 0
        if x_invert:
            mask |= 1  # Set bit 0
        if y_invert:
            mask |= 2  # Set bit 1
        if z_invert:
            mask |= 4  # Set bit 2

        self.serial_port.write(f'$23={mask}\n'.encode())
        time.sleep(0.5)
        response = self.serial_port.readline().decode('utf-8').strip()

        directions = []
        directions.append(f"X: {'positive' if x_invert else 'negative'}")
        directions.append(f"Y: {'positive' if y_invert else 'negative'}")
        directions.append(f"Z: {'positive' if z_invert else 'negative'}")

        print(f'Homing directions set - {", ".join(directions)}')
        print(f'Response: {response}')

    def check_and_clear_limit_switches(self):
        """Check if any limit switches are triggered and move off them."""
        print('Checking limit switch status...')
        self.serial_port.reset_input_buffer()
        self.serial_port.write(b'?\n')
        time.sleep(0.2)
        response = self.serial_port.readline().decode('utf-8').strip()
        print(f'Status: {response}')

        # Check if limit switch is triggered (look for "Lim" in status)
        if 'Lim' in response or 'Alarm' in response:
            print('WARNING: Limit switch is triggered!')
            print('Attempting to move off limit switch...')

            # Unlock the machine first
            self.serial_port.write(b'$X\n')
            time.sleep(0.5)

            # Disable hard limits temporarily to allow movement
            print('Temporarily disabling hard limits...')
            self.serial_port.write(b'$21=0\n')
            time.sleep(0.5)

            # Move away from switches in all axes
            print('Moving off switches...')
            self.serial_port.write(b'G91\n')  # Relative mode
            time.sleep(0.1)

            # Try moving in positive direction for all axes
            self.serial_port.write(b'G01 X5 Y5 Z2 F100\n')
            time.sleep(2)

            self.serial_port.write(b'G90\n')  # Absolute mode
            time.sleep(0.1)

            # Re-enable hard limits
            print('Re-enabling hard limits...')
            self.serial_port.write(b'$21=1\n')
            time.sleep(0.5)

            print('Moved off limit switches. Ready to home.')
            return True
        else:
            print('No limit switches triggered. Ready to home.')
            return False

    def move_to_machine_home(self):
        """Perform homing operation on the CNC machine, with X offset to avoid limit switch."""
        print("Starting homing sequence...")
        self.serial_port.write(b'$H\n')
        time.sleep(4)  # Give it time to complete homing

        # After homing, move X away from the limit switch by 2mm
        # This prevents X from sitting right on the limit switch at "home"
        print("Moving X to 2mm offset from limit switch...")
        self.serial_port.write(b'G90\n')  # Absolute mode
        time.sleep(0.1)
        self.serial_port.write(b'G0 X2\n')  # Move X to 2mm from machine home
        time.sleep(1)

        # Now reset the work coordinate system so this position (2mm from limit) becomes our new "home" (0,0,0)
        self.serial_port.write(b'G92 X0 Y0 Z0\n')  # Set current position as work coordinate 0,0,0
        time.sleep(0.1)

        self.xcoord = 0.0
        self.ycoord = 0.0
        self.zcoord = 0.0
        print('CNC machine homed successfully!')
        print('X is 2mm away from limit switch (set as X=0 in work coordinates)')
    
    def get_current_position_work_home(self):
        """Query CNC for current position relative to work home (G92)"""
        while True:
            self.serial_port.write(b'?\n')
            time.sleep(0.1)
            response = self.serial_port.readline().decode('utf-8').strip()
            print(f"Raw CNC response: {response}")
            
            match = re.search(r"WPos:([-\d.]+),([-\d.]+),([-\d.]+)", response)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                z = float(match.group(3))
                print(f"Current position (work home): X={x}, Y={y}, Z={z}")
                return x, y, z
            else:
                print("Warning: Unable to parse current position. Retrying...")
                time.sleep(0.1)
       
    def get_current_position_mechanical_home(self):
        """Query CNC for current position relative to mechanical home"""
        while True:
            self.serial_port.write(b'?\n')
            time.sleep(0.1)
            response = self.serial_port.readline().decode('utf-8').strip()
            print(f"Raw CNC response: {response}")
            
            match = re.search(r"MPos:([-\d.]+),([-\d.]+),([-\d.]+)", response)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                z = float(match.group(3))
                print(f"Current position (mechanical home): X={x}, Y={y}, Z={z}")
                return x, y, z
            else:
                print("Warning: Unable to parse current position. Retrying...")
                time.sleep(0.1)

    def get_feed_rate(self):
        """Get the current feed rate."""
        print(f'Current feed rate: {self.feed_rate} mm/min')
        return self.feed_rate
    
    def save_home_position(self, x, y, z):
        """Save the current home position to a file."""
        with open(self.HOME_FILE, "w") as f:
            f.write(f"{x},{y},{z}")
        print(f"Home position saved: X={x}, Y={y}, Z={z}")

    def load_home_position(self):
        """Load the saved home position from a file"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, self.HOME_FILE)
        print("file_path", file_path)
        if os.path.exists(file_path):
            try:
                with open(file_path, "r") as f:
                    data = f.read().strip()
                    x, y, z = map(float, data.split(","))
                    print(f"Loaded and set saved home position: X={x}, Y={y}, Z={z}")
                    return x, y, z
            except Exception as e:
                print(f"Error loading home position: {e}")
                print("Skipping home position load.")
        else:
            print("No saved home position found. Skipping home setup.")

    def set_curPos_home(self):
        """Set the current position as the work home and save it."""
        pos = self.get_current_position_mechanical_home()
        if pos:
            x, y, z = pos
            self.serial_port.write(b'G92 X0 Y0 Z0\n')
            time.sleep(0.1)
            # self.save_home_position(x, y, z)  # Save home position
            return x, y, z
        else:
            print("Error: Could not retrieve position to set home.")

    def relative_move(self, x, y, z):
        """Send a relative movement command to the CNC machine."""
        self.serial_port.write(f'G91 X{x} Y{y} Z{z}\n'.encode())
        time.sleep(0.1)

    def absolute_move(self, x, y, z):
        """Send an absolute movement command to the CNC machine."""
        self.serial_port.write(f'G90 G01 X{x} Y{y} Z{z} F{self.feed_rate}\n'.encode())
        time.sleep(0.1)
        print(f'Absolute Move: G90 X{x} Y{y} Z{z} F{self.feed_rate}')

    def query_status(self):
        """Query the current status of the CNC machine."""
        # Clear input buffer first to avoid reading stale data
        self.serial_port.reset_input_buffer()
        time.sleep(0.05)

        self.serial_port.write(b'?')
        time.sleep(0.1)  # Give GRBL time to respond

        # Read all available data, not just one line
        cnc_curr_status = ''
        while self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('<'):  # Status responses start with '<'
                cnc_curr_status = line
                break
            time.sleep(0.01)

        print(f'CNC status: {cnc_curr_status}')
        return cnc_curr_status

    def set_feed_rate(self, feed_rate):
        """Set the feed rate for CNC movement in mm/min."""
        self.feed_rate = feed_rate
        time.sleep(0.1)
        self.serial_port.write(f"F{feed_rate}\n".encode())
        print(f"Feed rate set to: {feed_rate} mm/min")
        time.sleep(0.1)

    def set_incremental_mode(self):
        """Set CNC to relative (incremental) positioning mode."""
        self.serial_port.write(b'G91\n')
        time.sleep(0.1)
        print("Incremental mode set.")
    
    def generate_sweep_path(self):
        """Generate dome sweep path coordinates."""
        coordinates = []
        
        x_start = 135.0
        x_end = 177.0
        x_peak = 156.0
        y = 0.0
        z_bottom = -27.0
        z_peak = -20.0
        z_transit = -17.0
        x_step = 2.0
        
        coordinates.append([x_start, y, z_transit])
        
        num_steps = int((x_end - x_start) / x_step) + 1
        
        for i in range(num_steps):
            x = x_start + (i * x_step)
            dist_from_peak = abs(x - x_peak)
            max_dist = max(x_peak - x_start, x_end - x_peak)
            z_dome = z_peak + (z_bottom - z_peak) * (dist_from_peak / max_dist) ** 2
            
            coordinates.append([x, y, z_dome])
            
            if i < num_steps - 1:
                coordinates.append([x, y, z_transit])
        
        coordinates.append([x_end, y, z_transit])
        
        return coordinates
    
    def sweep_dome(self, durability_tester=None):
        """Execute the dome sweep pattern with optional durability testing."""
        print("\n=== Starting Dome Sweep ===")
        print("Generating sweep path...")
        
        coordinates = self.generate_sweep_path()
        
        print(f"Generated {len(coordinates)} waypoints")
        print(f"First point: X={coordinates[0][0]}, Y={coordinates[0][1]}, Z={coordinates[0][2]}")
        print(f"Last point: X={coordinates[-1][0]}, Y={coordinates[-1][1]}, Z={coordinates[-1][2]}")
        
        self.serial_port.write(b'G90\n')
        time.sleep(0.1)
        self.set_feed_rate(200)
        
        total_points = len(coordinates)
        for idx, coord in enumerate(coordinates):
            x, y, z = coord
            print(f"[{idx+1}/{total_points}] Moving to X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            self.absolute_move(x, y, z)
            
            self.wait_for_idle()
            
            # Save data if durability tester is active
            if durability_tester and durability_tester.active:
                durability_tester.save_data_point()
            
            time.sleep(0.5)
        
        print("\n=== Sweep Complete ===")
        print("Returning to home position...")
        self.absolute_move(0, 0, 0)
        self.wait_for_idle()
        time.sleep(1)
        print("Done!")
    
    def repeat_test(self, durability_tester=None):
        """
        Repeat test: Move to X=147, Y=0, then alternate between Z=-18 and Z=-24 for 1000 cycles.
        Sends commands one at a time and waits for each to complete.
        """
        print("\n=== Starting Repeat Test ===")
        
        x = 147.0
        y = 0.0
        z_top = -18.0
        z_bottom = -24.0
        repetitions = 1000
        
        # Set absolute mode
        self.serial_port.write(b'G90\n')
        time.sleep(0.2)
        self.serial_port.reset_input_buffer()  # Clear buffer
        
        # Move to starting position with fast speed
        print(f"Moving to starting position: X={x}, Y={y}, Z={z_top}")
        cmd = f'G90 G01 X{x} Y{y} Z{z_top} F1500\n'
        print(f"[CMD] {cmd.strip()}")
        self.serial_port.write(cmd.encode())
        self.serial_port.flush()  # Ensure command is sent
        self.wait_for_idle()
        print("Arrived at starting position")
        
        print(f"\nStarting {repetitions} cycles of Z movement...")
        
        # Alternate between z_top and z_bottom
        for i in range(repetitions):
            # Move down to z_bottom
            cmd = f'G90 G01 X{x} Y{y} Z{z_bottom} F1500\n'
            print(f"[CMD {i+1}] DOWN: {cmd.strip()}")
            self.serial_port.write(cmd.encode())
            self.serial_port.flush()  # Ensure command is sent
            self.wait_for_idle()
            
            # Save data if durability tester is active
            if durability_tester and durability_tester.active:
                durability_tester.save_data_point()
            
            # Move up to z_top
            cmd = f'G90 G01 X{x} Y{y} Z{z_top} F1500\n'
            print(f"[CMD {i+1}] UP: {cmd.strip()}")
            self.serial_port.write(cmd.encode())
            self.serial_port.flush()  # Ensure command is sent
            self.wait_for_idle()
            
            # Print progress every 50 cycles
            if (i + 1) % 50 == 0:
                print(f"\n===== Progress: {i + 1}/{repetitions} cycles completed =====\n")
        
        print("\n=== Repeat Test Complete ===")
        print("Returning to home position...")
        cmd = f'G90 G01 X0 Y0 Z0 F1500\n'
        print(f"[CMD] {cmd.strip()}")
        self.serial_port.write(cmd.encode())
        self.serial_port.flush()  # Ensure command is sent
        self.wait_for_idle()
        time.sleep(1)
        print("Done!")
    
    def jog_mode(self, rotation_control=None):
        self.set_incremental_mode()
        self.set_feed_rate(200)
        lin_steps = [0.1, 0.5, 1, 2, 5, 10, 25, 50, 100]
        lin_i = 2
        rot_steps = [0.5, 1, 2, 5]
        rot_i = 1
        rot_rpm = 120
        mode = 1
        print("Jog → 1=translate 2=rotate  arrows X/Y  [ ] Z  ,/. lin‑step  ↑/↓ rot‑step Esc=q")

        def on_press(key):
            nonlocal lin_i, rot_i, mode
            try:
                ch = key.char.lower()
            except AttributeError:
                ch = None

            # Mode switching
            if ch == '1':
                mode = 1
                print("→ Translation mode")
                return
            if ch == '2':
                if not (rotation_control and rotation_control.serial_port):
                    print("(Rotation unavailable)")
                    return
                mode = 2
                print("→ Rotation mode")
                return

            # Quit
            if key in (keyboard.Key.esc,) or ch == 'q':
                print("Exiting jog")
                return False
        
            # Translation controls
            if mode == 1:
                if key == keyboard.Key.right:
                    amt = lin_steps[lin_i]
                    self.relative_move(amt, 0, 0)
                    print(f"X +{amt}")
                elif key == keyboard.Key.left:
                    amt = lin_steps[lin_i]
                    self.relative_move(-amt, 0, 0)
                    print(f"X -{amt}")
                elif key == keyboard.Key.up:
                    amt = lin_steps[lin_i]
                    self.relative_move(0, amt, 0)
                    print(f"Y +{amt}")
                elif key == keyboard.Key.down:
                    amt = lin_steps[lin_i]
                    self.relative_move(0, -amt, 0)
                    print(f"Y -{amt}")
                elif ch == '[':
                    amt = lin_steps[lin_i]
                    self.relative_move(0, 0, -amt)
                    print(f"Z -{amt}")
                elif ch == ']':
                    amt = lin_steps[lin_i]
                    self.relative_move(0, 0, amt)
                    print(f"Z +{amt}")
                elif ch == ',':
                    lin_i = max(0, lin_i - 1)
                    print(f"Step {lin_steps[lin_i]} mm")
                elif ch == '.':
                    lin_i = min(len(lin_steps) - 1, lin_i + 1)
                    print(f"Step {lin_steps[lin_i]} mm")

            # Rotation controls
            elif mode == 2:
                if key == keyboard.Key.left:
                    deg = rot_steps[rot_i]
                    rotation_control.send_step("CCW", deg, rot_rpm)
                    print(f"CCW {deg}°")
                elif key == keyboard.Key.right:
                    deg = rot_steps[rot_i]
                    rotation_control.send_step("CW", deg, rot_rpm)
                    print(f"CW {deg}°")
                elif key == keyboard.Key.up:
                    rot_i = min(len(rot_steps) - 1, rot_i + 1)
                    print(f"Rot step {rot_steps[rot_i]}°")
                elif key == keyboard.Key.down:
                    rot_i = max(0, rot_i - 1)
                    print(f"Rot step {rot_steps[rot_i]}°")

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    def test_mode(self, l, segments, wait_t, z_offset):
        """Sweep a square centered at the current work-home origin."""
        self.set_incremental_mode()
        vel = 200
        self.set_feed_rate(vel)

        dz = float(z_offset)
        z = 0.0
        if abs(dz) > 1e-9:
            self.relative_move(0, 0, dz)
            time.sleep(wait_t)
            z += dz

        step_xy = l / (segments - 1) if segments > 1 else 0.0

        x = 0.0
        y = 0.0
        self.relative_move(-l/2, l/2, 0)
        time.sleep(wait_t)
        x += -l/2
        y += l/2
        print(f"Current Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

        counter = 0
        total_moves = (segments - 1) * segments
        for row in range(segments):
            dir_sign = 1 if row % 2 == 0 else -1

            for col in range(segments - 1):
                dx = dir_sign * step_xy
                self.relative_move(dx, 0, 0)
                time.sleep(wait_t)
                x += dx
                counter += 1
                print(f"Progress: {counter / total_moves * 100:.2f}%  Current Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

            if row < segments - 1:
                dy = -step_xy
                self.relative_move(0, dy, 0)
                time.sleep(wait_t)
                y += dy
                print(f"Current Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

        self.relative_move(-x, -y, 0)
        print("Test-mode square complete.")
    
    def play_sound(self):
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            wav_path = os.path.join(script_dir, "truck_sound.wav")
            wave_obj = sa.WaveObject.from_wave_file(wav_path)
            play_obj = wave_obj.play()
            play_obj.wait_done()
            print("Played truck_sound.wav")
        except Exception as e:
            print(f"Error playing sound: {e}")

    def test_rotation(self, rotation_control=None, step_size=1, step_count=5, wait_time=4):
        """Test rotation of the CNC machine."""
        total_steps = step_count * 4
        count = 0
        if not rotation_control or not rotation_control.serial_port:
            print("Rotation control unavailable.")
            return

        for i in range(step_count):
            rotation_control.send_step("CW", step_size, 60)
            count += 1
            print(f"Progress: {count / total_steps * 100:.2f}%")
            time.sleep(wait_time)

        for i in range(step_count * 2):
            rotation_control.send_step("CCW", step_size, 60)
            count += 1
            print(f"Progress: {count / total_steps * 100:.2f}%")
            time.sleep(wait_time)

        for i in range(step_count):
            rotation_control.send_step("CW", step_size, 60)
            count += 1
            print(f"Progress: {count / total_steps * 100:.2f}%")
            time.sleep(wait_time)

        time.sleep(1)

    def kill_serial_connection(self):
        """Close the serial connection to the CNC controller."""
        if hasattr(self, 'serial_port') and self.serial_port:
            if self.serial_port.is_open:
                self.serial_port.close()
                print("CNC serial connection closed.")
            else:
                print("CNC serial connection already closed.")
        else:
            print("No serial connection to close.")
