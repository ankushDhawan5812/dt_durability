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
from durability_tester import DurabilityTester
from arduino_controller import ArduinoController
from cnc_control import CNCController

import serial.tools.list_ports
import NetFT

RUN_HOMING_SEQUENCE = False

X_HOME_OFFSET = 147.0
Z_HOME_OFFSET = -17.0

def main():
    # Create controller instances
    controller = CNCController()
    controller_rotation = ArduinoController()
    durability_tester = DurabilityTester()

    # Unlock CNC and perform homing
    controller.unlock_cnc()
    controller.set_feed_rate(200)

    if RUN_HOMING_SEQUENCE: 
        # Configure homing directions
        # Set X to home in POSITIVE direction (where your limit switch is)
        # Keep Y and Z in their default directions
        # Change x_invert to False if your X limit is in the negative direction
        controller.set_homing_direction(x_invert=True, y_invert=False, z_invert=False)

        controller.move_to_machine_home() # home CNC
        controller.wait_for_idle() # wait for up to 100 seconds while it becomes idle
        controller.query_status()

        # Unlock CNC (clears alarm state from homing)
        controller.unlock_cnc()
        time.sleep(1.0)  # Wait longer for unlock to process

    # Set up CNC for movement
    controller.set_incremental_mode()  # Set to relative mode for test move

    # Set back to absolute mode for normal operation
    controller.serial_port.write(b'G90\n')
    time.sleep(0.1)

    while True:
        command = input("Enter command (s=status, mh=move to machine home, h=go to work home, sethome=set work home, gp=get position, a=absolute move, r=relative move, j=jog mode, rt=run_durability_test, eh=move to experiment home, q=quit): ").strip().lower()

        if command == 's':
            controller.query_status()

        elif command == 'u':
            controller.unlock_cnc()

        elif command == 'clear_limit':
            controller.check_and_clear_limit_switches()

        elif command == 'mh':
            controller.move_to_machine_home()
        
        elif command == 'h':
            x, y, z = controller.load_home_position()
            controller.set_feed_rate(1000)
            controller.absolute_move(x, y, z)
        
        elif command == 'eh':
            # move to experiment home
            controller.relative_move(X_HOME_OFFSET,0,Z_HOME_OFFSET)
        
        elif command == 'f':
            force = durability_tester.get_force_reading()
            print(force)
        
        elif command == 'rt':
            durability_tester.run_durability_test(controller)

        elif command == 'sethome':
            x, y, z = controller.get_current_position_mechanical_home()
            controller.save_home_position(x, y, z)
            controller.set_curPos_home()

        elif command == 'p':
            controller.play_sound()
        
        elif command == 'gp':
            controller.get_current_position_mechanical_home()
        
        elif command == 'swp':
            controller.sweep_dome(durability_tester)
        
        elif command == 'rpt':
            controller.repeat_test(durability_tester)
        
        elif command == 'a':
            try:
                coordinates = input("Enter the target position (x,y,z): ").strip().split(',')
                if len(coordinates) == 3:
                    x = float(coordinates[0])
                    y = float(coordinates[1])
                    z = float(coordinates[2])
                    controller.absolute_move(x, y, z)
                else:
                    print("Invalid input. Please enter coordinates in the format x,y,z.")
            except ValueError:
                print("Invalid input. Please enter numeric values for x, y, and z.")
        
        elif command == 'r':
            try:
                distances = input("Enter the distance to move (x,y,z): ").strip().split(',')
                if len(distances) == 3:
                    x = float(distances[0])
                    y = float(distances[1])
                    z = float(distances[2])
                    controller.relative_move(x, y, z)
                else:
                    print("Invalid input. Please enter distances in the format x,y,z.")
            except ValueError:
                print("Invalid input. Please enter numeric values for x, y, and z.")
        
        elif command == 'j':
            controller.jog_mode(rotation_control=controller_rotation)
        
        elif command == "test":
            controller.test_rotation(controller_rotation, step_size=2, step_count=5, wait_time=4)
            controller.play_sound()
        
        elif command.startswith("test "):
            try:
                _, argstr = command.split(" ", 1)
                l, seg, wt, z = [a.strip() for a in argstr.split(",")]
                controller.test_mode(float(l), int(seg), float(wt), z)
            except Exception as e:
                print(f"Error: {e}")
                print("Usage: test l,segments,wait_time,z")
        
        elif command == 'dt_start':
            durability_tester.initialize()
        
        elif command == 'dt_save':
            durability_tester.save_data_point()
        
        elif command == 'dt_tare':
            durability_tester.tare_sensor()
        
        elif command == 'dt_stop':
            durability_tester.shutdown()
        
        elif command == 'q':
            print("Exiting program.")
            durability_tester.shutdown()
            break
        
        else:
            print("Invalid command. Try again.")


if __name__ == '__main__':
    main()