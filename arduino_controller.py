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

class ArduinoController:
    """Detect Arduino (skip GRBL/CNC ports) and send step-motor commands."""
    _KNOWN_VIDS = {0x2341, 0x1A86, 0x10C4, 0x1B4F}
    _CNC_TAGS = ("grbl", "cnc")
    _PREFERRED_PATH = "/dev/serial/by-id/"

    def __init__(self, baudrate: int = 115200, timeout: float = 1):
        self.port = self._find_port()
        if not self.port:
            self.serial_port = None
            print("(Rotation unavailable — no Arduino detected.)")
            return
        self.serial_port = serial.Serial(self.port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)
        print(f"Arduino connected on {self.port}")

    def _find_port(self):
        # Prefer using by-id path for more reliable detection
        if os.path.exists(self._PREFERRED_PATH):
            for fname in os.listdir(self._PREFERRED_PATH):
                path = os.path.join(self._PREFERRED_PATH, fname)
                if "arduino" in fname.lower() and os.path.exists(path):
                    return os.path.realpath(path)

        # Fallback: Use VID/manufacturer/descriptions
        for p in serial.tools.list_ports.comports():
            desc = p.description.lower()
            manu = (p.manufacturer or "").lower()
            if any(tag in desc for tag in self._CNC_TAGS):
                continue
            if p.vid in self._KNOWN_VIDS or "arduino" in desc or "arduino" in manu:
                return p.device

        return None

    def send_step(self, direction: str, deg: float, rpm: int = 120):
        if not self.serial_port:
            return
        cmd = f"{direction.upper()} {deg} {rpm}\n"
        self.serial_port.write(cmd.encode())
        print(f"[Arduino] → {direction.upper()} {deg}°")

    def close(self):
        if self.serial_port:
            self.serial_port.close()