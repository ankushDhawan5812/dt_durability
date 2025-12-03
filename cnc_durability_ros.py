#!/usr/bin/env python3

"""
ROS2-based CNC Durability Testing System
Separate nodes for ATI sensor, image capture, and CNC control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
import serial
import time
import os
import glob
import numpy as np
import platform
import cv2
import csv
from skimage.metrics import structural_similarity as ssim
import serial.tools.list_ports
import NetFT

##############################################################################
# CONFIGURATION
##############################################################################
# Camera Settings
CAMERA_DEVICE_ID = 4
EXPOSURE_VALUE = 50
WB_TEMP = 6500
FRAME_WIDTH = 1600
FRAME_HEIGHT = 1200

# ATI Sensor Settings
SENSOR_IP = "192.168.2.1"
COUNTS_PER_FORCE = 1000000.0
COUNTS_PER_TORQUE = 1000000.0

# Durability Test Settings
FORCE_TRIGGER_THRESHOLD = 7.0  # Newtons (force to trigger image capture)
FORCE_ZERO_THRESHOLD = 0.5     # Newtons (threshold to consider force as "zero")
CNC_COMMAND_DELAY = 0.25       # Seconds between CNC commands
##############################################################################


class ATISensorNode(Node):
    """
    ROS2 Node: Continuously reads and publishes ATI force/torque sensor data
    """
    def __init__(self):
        super().__init__('ati_sensor_node')

        # Publisher for force/torque data
        self.force_publisher = self.create_publisher(
            WrenchStamped,
            '/ati_sensor/wrench',
            10
        )

        # Parameters
        self.declare_parameter('sensor_ip', SENSOR_IP)
        self.declare_parameter('publish_rate', 100.0)  # Hz

        sensor_ip = self.get_parameter('sensor_ip').value
        publish_rate = self.get_parameter('publish_rate').value

        # Initialize ATI Sensor
        self.get_logger().info(f'Connecting to ATI Sensor at {sensor_ip}...')
        try:
            self.sensor = NetFT.Sensor(sensor_ip)
            self.sensor.tare()
            self.sensor.startStreaming()
            self.get_logger().info('ATI Sensor connected and streaming')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ATI sensor: {e}')
            raise

        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_force_data)

    def publish_force_data(self):
        """Read sensor and publish force/torque data"""
        raw_ft = self.sensor.measurement()
        if raw_ft:
            # Scale to Newtons and Newton-meters
            fx = raw_ft[0] / COUNTS_PER_FORCE
            fy = raw_ft[1] / COUNTS_PER_FORCE
            fz = raw_ft[2] / COUNTS_PER_FORCE
            tx = raw_ft[3] / COUNTS_PER_TORQUE
            ty = raw_ft[4] / COUNTS_PER_TORQUE
            tz = raw_ft[5] / COUNTS_PER_TORQUE

            # Create and publish message
            msg = WrenchStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'ati_sensor'
            msg.wrench.force.x = fx
            msg.wrench.force.y = fy
            msg.wrench.force.z = fz
            msg.wrench.torque.x = tx
            msg.wrench.torque.y = ty
            msg.wrench.torque.z = tz

            self.force_publisher.publish(msg)

    def tare_sensor(self):
        """Re-tare the sensor"""
        self.get_logger().info('Taring sensor...')
        self.sensor.stopStreaming()
        time.sleep(0.1)
        self.sensor.tare()
        self.sensor.startStreaming()
        self.get_logger().info('Sensor tared')

    def shutdown(self):
        """Clean shutdown"""
        self.sensor.stopStreaming()


class DTImageNode(Node):
    """
    ROS2 Node: Captures images when force threshold is reached
    Triggers on rising above threshold and falling back to zero
    """
    def __init__(self):
        super().__init__('dt_image_node')

        # Parameters
        self.declare_parameter('camera_device_id', CAMERA_DEVICE_ID)
        self.declare_parameter('force_trigger_threshold', FORCE_TRIGGER_THRESHOLD)
        self.declare_parameter('force_zero_threshold', FORCE_ZERO_THRESHOLD)
        self.declare_parameter('save_root', 'durability_test')

        self.camera_id = self.get_parameter('camera_device_id').value
        self.force_trigger = self.get_parameter('force_trigger_threshold').value
        self.force_zero = self.get_parameter('force_zero_threshold').value

        # State management
        self.force_z = 0.0
        self.last_force_z = 0.0
        self.current_wrench = None
        self.above_threshold = False
        self.initial_image = None
        self.trial_dir = None
        self.csv_file = None
        self.csv_writer = None
        self.active = False

        # Subscribe to force data
        self.force_subscription = self.create_subscription(
            WrenchStamped,
            '/ati_sensor/wrench',
            self.force_callback,
            10
        )

        # Publisher for capture status
        self.status_publisher = self.create_publisher(
            String,
            '/dt_image/status',
            10
        )

        self.get_logger().info('DT Image Node initialized (waiting for start command)')

    def initialize_test(self):
        """Initialize camera and data logging"""
        if self.active:
            self.get_logger().warn('Test already initialized')
            return False

        self.get_logger().info('Initializing durability test...')

        # Initialize Camera
        self.init_camera()

        # Setup data directory
        base_dir = os.path.dirname(os.path.abspath(__file__))
        save_root = os.path.join(base_dir, self.get_parameter('save_root').value)
        self.trial_dir = self.get_next_trial_dir(save_root)
        self.csv_path = os.path.join(self.trial_dir, 'data_log.csv')

        # Initialize CSV
        self.init_csv()

        # Capture initial reference image
        ret, frame = self.cap.read()
        if ret:
            self.initial_image = frame.copy()
            cv2.imwrite(os.path.join(self.trial_dir, "initial_image.png"), self.initial_image)
            self.get_logger().info('Initial reference image captured')
        else:
            self.get_logger().error('Failed to capture initial image')
            return False

        self.active = True
        self.get_logger().info(f'Test initialized. Data will be saved to: {self.trial_dir}')
        return True

    def init_camera(self):
        """Initialize camera with manual settings"""
        self.get_logger().info(f'Opening camera /dev/video{self.camera_id}...')
        self.cap = cv2.VideoCapture(self.camera_id)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Manual camera settings
        self.set_manual_exposure(self.camera_id, EXPOSURE_VALUE)
        self.set_manual_wb(self.camera_id, WB_TEMP)

        time.sleep(2)
        for _ in range(10):
            self.cap.read()

        self.get_logger().info('Camera initialized')

    def set_manual_exposure(self, video_id, exposure_time):
        """Set manual exposure using v4l2-ctl"""
        commands = [
            f"v4l2-ctl --device /dev/video{video_id} -c auto_exposure=3",
            f"v4l2-ctl --device /dev/video{video_id} -c auto_exposure=1",
            f"v4l2-ctl --device /dev/video{video_id} -c exposure_time_absolute={exposure_time}"
        ]
        for cmd in commands:
            os.system(cmd)

    def set_manual_wb(self, video_id, wb_temp):
        """Set manual white balance using v4l2-ctl"""
        commands = [
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_automatic=0",
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_automatic=1",
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_automatic=0",
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_temperature={wb_temp}"
        ]
        for cmd in commands:
            os.system(cmd)

    def get_next_trial_dir(self, save_root):
        """Create numbered trial directory"""
        if not os.path.exists(save_root):
            os.makedirs(save_root)

        i = 1
        while True:
            trial_dir = os.path.join(save_root, f"trial_{i}")
            if not os.path.exists(trial_dir):
                os.makedirs(trial_dir)
                return trial_dir
            i += 1

    def init_csv(self):
        """Initialize CSV file"""
        headers = [
            'timestamp', 'image_filename',
            'fx', 'fy', 'fz', 'tx', 'ty', 'tz',
            'mse', 'psnr', 'ssim', 'l1_error'
        ]
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(headers)
        self.csv_file.flush()

    def force_callback(self, msg):
        """Monitor force and trigger image capture"""
        if not self.active:
            return

        self.last_force_z = self.force_z
        self.force_z = msg.wrench.force.z
        self.current_wrench = msg

        # Detect rising edge (force crosses threshold going up)
        if not self.above_threshold and self.force_z >= self.force_trigger:
            self.get_logger().info(f'Force threshold reached: {self.force_z:.2f}N - Capturing image')
            self.capture_image('rising')
            self.above_threshold = True

        # Detect falling edge (force returns to near zero)
        elif self.above_threshold and self.force_z <= self.force_zero:
            self.get_logger().info(f'Force returned to zero: {self.force_z:.2f}N - Capturing image')
            self.capture_image('falling')
            self.above_threshold = False

    def capture_image(self, trigger_type):
        """Capture and save image with sensor data"""
        if not self.active:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")

        # Compute metrics
        mse, psnr, ssim_val, l1 = self.compute_metrics(frame, self.initial_image)

        # Save image
        img_filename = f"image_{timestamp}_{trigger_type}.png"
        img_path = os.path.join(self.trial_dir, img_filename)
        cv2.imwrite(img_path, frame)

        # Get force data
        if self.current_wrench:
            fx = self.current_wrench.wrench.force.x
            fy = self.current_wrench.wrench.force.y
            fz = self.current_wrench.wrench.force.z
            tx = self.current_wrench.wrench.torque.x
            ty = self.current_wrench.wrench.torque.y
            tz = self.current_wrench.wrench.torque.z
        else:
            fx = fy = fz = tx = ty = tz = 0.0

        # Save to CSV
        row = [
            timestamp, img_filename,
            f"{fx:.4f}", f"{fy:.4f}", f"{fz:.4f}",
            f"{tx:.4f}", f"{ty:.4f}", f"{tz:.4f}",
            f"{mse:.4f}", f"{psnr:.4f}", f"{ssim_val:.4f}", f"{l1:.4f}"
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        # Publish status
        status_msg = String()
        status_msg.data = f"captured_{trigger_type}"
        self.status_publisher.publish(status_msg)

        self.get_logger().info(f'Saved: {img_filename} | Fz: {fz:.2f}N | SSIM: {ssim_val:.4f}')

    def compute_metrics(self, img1, img2):
        """Compute image similarity metrics"""
        g1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        g2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        diff = (g1.astype(np.float32) - g2.astype(np.float32)) ** 2
        mse = np.mean(diff)

        if mse == 0:
            psnr = 100.0
        else:
            psnr = 10 * np.log10(255.0**2 / mse)

        l1_error = np.mean(np.abs(g1.astype(np.float32) - g2.astype(np.float32)))
        score, _ = ssim(g1, g2, full=True)

        return mse, psnr, score, l1_error

    def shutdown(self):
        """Clean shutdown"""
        if self.active:
            if self.cap:
                self.cap.release()
            if self.csv_file:
                self.csv_file.close()
            self.active = False


class CNCControlNode(Node):
    """
    ROS2 Node: Controls CNC machine with GRBL
    Adds configurable delay between commands
    """
    def __init__(self):
        super().__init__('cnc_control_node')

        # Parameters
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('command_delay', CNC_COMMAND_DELAY)
        self.declare_parameter('feed_rate', 500)

        self.command_delay = self.get_parameter('command_delay').value
        self.feed_rate = self.get_parameter('feed_rate').value

        # Connect to CNC
        self.serial_port = self.connect_to_cnc()
        if not self.serial_port:
            raise Exception("Failed to connect to CNC")

        # Last command timestamp
        self.last_command_time = time.time()

        # Subscribe to movement commands
        self.command_subscription = self.create_subscription(
            String,
            '/cnc/command',
            self.command_callback,
            10
        )

        # Publisher for status
        self.status_publisher = self.create_publisher(
            String,
            '/cnc/status',
            10
        )

        self.get_logger().info('CNC Control Node initialized')

    def connect_to_cnc(self):
        """Connect to CNC controller"""
        self.get_logger().info('Connecting to CNC...')

        system = platform.system()
        if system == "Darwin":
            available_ports = glob.glob("/dev/cu.usb*") + glob.glob("/dev/tty.usb*")
            available_ports = [p for p in available_ports if "wlan" not in p.lower() and "bluetooth" not in p.lower()]
        elif system == "Linux":
            available_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        else:
            available_ports = [p.device for p in serial.tools.list_ports.comports()]

        for port in available_ports:
            try:
                self.get_logger().info(f'Trying {port}...')
                ser = serial.Serial(port=port, baudrate=115200, timeout=1)
                time.sleep(2)
                self.get_logger().info(f'Connected to CNC on {port}')
                return ser
            except serial.SerialException:
                continue

        self.get_logger().error('No CNC found')
        return None

    def wait_for_idle(self):
        """Wait for CNC to finish movement"""
        time.sleep(0.05)
        self.serial_port.reset_input_buffer()

        while True:
            self.serial_port.write(b'?')
            time.sleep(0.05)
            response = self.serial_port.readline().decode('utf-8').strip()

            if response and (response.startswith("<Idle") or "Idle" in response):
                break
            time.sleep(0.1)

    def enforce_command_delay(self):
        """Ensure minimum delay between commands"""
        elapsed = time.time() - self.last_command_time
        if elapsed < self.command_delay:
            delay_needed = self.command_delay - elapsed
            self.get_logger().debug(f'Enforcing {delay_needed:.3f}s delay')
            time.sleep(delay_needed)
        self.last_command_time = time.time()

    def command_callback(self, msg):
        """Process CNC movement commands"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Enforce delay before executing command
        self.enforce_command_delay()

        # Parse and execute command
        if command.startswith('absolute'):
            # Format: "absolute x,y,z"
            _, coords = command.split(' ', 1)
            x, y, z = map(float, coords.split(','))
            self.absolute_move(x, y, z)
        elif command.startswith('relative'):
            # Format: "relative x,y,z"
            _, coords = command.split(' ', 1)
            x, y, z = map(float, coords.split(','))
            self.relative_move(x, y, z)
        elif command == 'home':
            self.move_to_home()
        elif command == 'set_home':
            self.set_current_as_home()
        elif command == 'unlock':
            self.unlock_cnc()
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def absolute_move(self, x, y, z):
        """Send absolute movement command"""
        cmd = f'G90 G01 X{x} Y{y} Z{z} F{self.feed_rate}\n'
        self.get_logger().info(f'Moving to X={x}, Y={y}, Z={z}')
        self.serial_port.write(cmd.encode())
        time.sleep(0.1)
        self.wait_for_idle()

        status_msg = String()
        status_msg.data = f'arrived_{x}_{y}_{z}'
        self.status_publisher.publish(status_msg)

    def relative_move(self, x, y, z):
        """Send relative movement command"""
        cmd = f'G91 X{x} Y{y} Z{z}\n'
        self.serial_port.write(cmd.encode())
        time.sleep(0.1)
        self.wait_for_idle()

    def move_to_home(self):
        """Move to home position"""
        self.absolute_move(0, 0, 0)

    def set_current_as_home(self):
        """Set current position as home"""
        self.serial_port.write(b'G92 X0 Y0 Z0\n')
        time.sleep(0.1)
        self.get_logger().info('Current position set as home')

    def unlock_cnc(self):
        """Unlock CNC"""
        self.serial_port.write(b'$X\n')
        time.sleep(1)
        self.serial_port.write(b"$10=0\n")
        time.sleep(0.1)
        self.get_logger().info('CNC unlocked')

    def shutdown(self):
        """Clean shutdown"""
        if self.serial_port:
            self.serial_port.close()


class DurabilityTestCoordinator(Node):
    """
    Main coordinator node for durability testing
    Manages test execution and coordinates other nodes
    """
    def __init__(self):
        super().__init__('durability_coordinator')

        # Publisher for CNC commands
        self.cnc_command_publisher = self.create_publisher(
            String,
            '/cnc/command',
            10
        )

        # Subscribe to image capture status
        self.image_status_subscription = self.create_subscription(
            String,
            '/dt_image/status',
            self.image_status_callback,
            10
        )

        # Subscribe to CNC status
        self.cnc_status_subscription = self.create_subscription(
            String,
            '/cnc/status',
            self.cnc_status_callback,
            10
        )

        self.get_logger().info('Durability Test Coordinator initialized')
        self.test_running = False

    def image_status_callback(self, msg):
        """Handle image capture status updates"""
        self.get_logger().info(f'Image capture status: {msg.data}')

    def cnc_status_callback(self, msg):
        """Handle CNC status updates"""
        self.get_logger().info(f'CNC status: {msg.data}')

    def send_cnc_command(self, command):
        """Send command to CNC"""
        msg = String()
        msg.data = command
        self.cnc_command_publisher.publish(msg)
        self.get_logger().info(f'Sent CNC command: {command}')

    def run_repeat_test(self, x=147.0, y=0.0, z_top=-18.0, z_bottom=-24.0, cycles=1000):
        """Run repeat test with automatic image capture"""
        self.get_logger().info(f'Starting repeat test: {cycles} cycles')

        # Move to starting position
        self.send_cnc_command(f'absolute {x},{y},{z_top}')
        time.sleep(2)

        # Execute cycles
        for i in range(cycles):
            # Move down (will trigger image on high force)
            self.send_cnc_command(f'absolute {x},{y},{z_bottom}')
            time.sleep(1)

            # Move up (will trigger image on low force)
            self.send_cnc_command(f'absolute {x},{y},{z_top}')
            time.sleep(1)

            if (i + 1) % 50 == 0:
                self.get_logger().info(f'Progress: {i+1}/{cycles} cycles completed')

        # Return home
        self.send_cnc_command('home')
        self.get_logger().info('Repeat test complete')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    print("\n" + "="*60)
    print("ROS2 CNC Durability Testing System")
    print("="*60)
    print("\nStarting nodes...")

    try:
        # Create nodes
        ati_node = ATISensorNode()
        dt_image_node = DTImageNode()
        cnc_node = CNCControlNode()
        coordinator = DurabilityTestCoordinator()

        # Initialize test
        print("\nInitializing durability test...")
        cnc_node.unlock_cnc()
        cnc_node.set_current_as_home()
        dt_image_node.initialize_test()

        print("\nAll nodes ready!")
        print("\nTopics:")
        print("  /ati_sensor/wrench    - Force/torque data")
        print("  /cnc/command          - Send CNC commands")
        print("  /cnc/status           - CNC status updates")
        print("  /dt_image/status      - Image capture status")
        print("\nUse coordinator.run_repeat_test() or send commands via /cnc/command")
        print("="*60 + "\n")

        # Create executor to spin all nodes
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(ati_node)
        executor.add_node(dt_image_node)
        executor.add_node(cnc_node)
        executor.add_node(coordinator)

        try:
            executor.spin()
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            ati_node.shutdown()
            dt_image_node.shutdown()
            cnc_node.shutdown()

            ati_node.destroy_node()
            dt_image_node.destroy_node()
            cnc_node.destroy_node()
            coordinator.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
