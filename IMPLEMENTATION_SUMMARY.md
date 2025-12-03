# CNC Durability ROS Implementation Summary

## Overview
Created `cnc_durability_ros.py` - a ROS2-based version of the durability testing system with distributed node architecture.

## Key Implementation Details

### 1. ATI Sensor Node (Lines 44-120)
**Purpose**: Continuously publishes force/torque data

**Key Features**:
- Publishes at 100 Hz to `/ati_sensor/wrench` topic
- Force values automatically scaled to Newtons
- Torque values in Newton-meters
- Uses `WrenchStamped` message type with proper timestamps
- Tare functionality available

**ROS Topic**:
```
/ati_sensor/wrench (geometry_msgs/WrenchStamped)
  - wrench.force.x/y/z (Newtons)
  - wrench.torque.x/y/z (Newton-meters)
```

### 2. DT Image Node (Lines 122-369)
**Purpose**: Force-triggered image capture with automatic detection

**Trigger Logic** (Lines 244-262):
```python
# Rising edge: Force crosses threshold going UP
if not self.above_threshold and self.force_z >= self.force_trigger:
    self.capture_image('rising')
    self.above_threshold = True

# Falling edge: Force returns to zero going DOWN
elif self.above_threshold and self.force_z <= self.force_zero:
    self.capture_image('falling')
    self.above_threshold = False
```

**Configurable Thresholds**:
- `force_trigger_threshold`: 7.0 N (default) - triggers on contact
- `force_zero_threshold`: 0.5 N (default) - triggers on release
- Both adjustable via ROS parameters

**Data Captured**:
- Timestamped PNG images (named with `_rising` or `_falling` suffix)
- Force/torque at capture moment
- Image metrics: MSE, PSNR, SSIM, L1 error
- All logged to CSV

**ROS Topics**:
- Subscribes: `/ati_sensor/wrench`
- Publishes: `/dt_image/status` (capture events)

### 3. CNC Control Node (Lines 371-522)
**Purpose**: CNC movement control with enforced delays

**Command Delay Implementation** (Lines 472-480):
```python
def enforce_command_delay(self):
    """Ensure minimum delay between commands"""
    elapsed = time.time() - self.last_command_time
    if elapsed < self.command_delay:
        delay_needed = self.command_delay - elapsed
        time.sleep(delay_needed)
    self.last_command_time = time.time()
```

**Key Feature**:
- **0.25 second delay** enforced before EVERY command execution
- Prevents CNC controller overload
- Configurable via `command_delay` parameter

**Supported Commands**:
- `absolute x,y,z` - Absolute positioning
- `relative x,y,z` - Relative movement
- `home` - Return to (0,0,0)
- `set_home` - Set current as home
- `unlock` - Unlock CNC

**ROS Topics**:
- Subscribes: `/cnc/command`
- Publishes: `/cnc/status`

### 4. Coordinator Node (Lines 524-583)
**Purpose**: High-level test orchestration

**Features**:
- `run_repeat_test()` method for automated cycling
- Monitors image capture and CNC status
- Sends commands to CNC node
- Reports progress

## How Force-Triggered Capture Works

```
Force Timeline:

8 N  │     ┌────┐
     │    ╱│    │╲        Rising edge detected (7.0 N)
7 N ─┼───╱─┼────┼─╲──     → Image captured (contact)
     │  ╱  │    │  ╲
     │ ╱   │    │   ╲
     │╱    │    │    ╲
0 N ─┴─────┴────┴─────╲── Falling edge detected (< 0.5 N)
                        → Image captured (release)
     t0    t1   t2    t3

Image Files:
- image_YYYYMMDD_HHMMSS_rising.png  (captured at t1)
- image_YYYYMMDD_HHMMSS_falling.png (captured at t3)
```

## Communication Flow

```
┌─────────────────┐
│  ATI Sensor     │  Publishes force @ 100 Hz
│  Node           │────────────────┐
└─────────────────┘                │
                                   ▼
                         ┌─────────────────┐
                         │  DT Image       │
                         │  Node           │ Monitors force,
                         │                 │ captures images
                         └─────────────────┘
                                   │
                                   │ Status updates
                                   ▼
┌─────────────────┐      ┌─────────────────┐
│  CNC Control    │◄─────│  Coordinator    │
│  Node           │      │  Node           │
│                 │      │                 │
│  (0.25s delay)  │      └─────────────────┘
└─────────────────┘

Topic Flow:
/ati_sensor/wrench ──→ DT Image Node
                  ──→ (available to all nodes)

/cnc/command ──→ CNC Control Node
/cnc/status ──→ Coordinator

/dt_image/status ──→ Coordinator
```

## Configuration Summary

### Force Thresholds
| Parameter | Default | Purpose |
|-----------|---------|---------|
| `FORCE_TRIGGER_THRESHOLD` | 7.0 N | Trigger capture on contact |
| `FORCE_ZERO_THRESHOLD` | 0.5 N | Trigger capture on release |

### Timing
| Parameter | Default | Purpose |
|-----------|---------|---------|
| `CNC_COMMAND_DELAY` | 0.25 s | Delay between CNC commands |
| ATI Publish Rate | 100 Hz | Force data frequency |

### Camera
| Parameter | Default |
|-----------|---------|
| Resolution | 1600x1200 |
| Exposure | 50 (manual) |
| White Balance | 6500K |

## Usage Example

### Running Automated Test
```python
# Start the system
python3 cnc_durability_ros.py

# In the coordinator, run test
coordinator.run_repeat_test(
    x=147.0,
    y=0.0,
    z_top=-18.0,    # No contact position
    z_bottom=-24.0, # Contact position (will exceed 7N)
    cycles=1000
)
```

### What Happens:
1. CNC moves to (147, 0, -18) - starting position
2. For each cycle:
   - Move down to z=-24 (contacts sensor)
   - Force rises → triggers image capture when > 7N
   - **Wait 0.25s** before next command
   - Move up to z=-18 (releases sensor)
   - Force falls → triggers image capture when < 0.5N
   - **Wait 0.25s** before next command
3. After 1000 cycles, return home
4. All images and data saved to `durability_test/trial_N/`

## Key Improvements Over Original

1. **Modular Design**: Independent nodes can be debugged/modified separately
2. **Automatic Triggering**: No manual 's' key press needed
3. **Continuous Monitoring**: Force always available to all nodes
4. **Enforced Delays**: Guaranteed 0.25s between CNC commands
5. **Dual Triggers**: Captures both contact and release moments
6. **Configurable**: All thresholds adjustable via ROS parameters
7. **Status Reporting**: Real-time updates via ROS topics
8. **Scalable**: Can run nodes on different computers

## Files Created

1. `cnc_durability_ros.py` - Main ROS2 node implementation
2. `package.xml` - ROS2 package metadata
3. `launch_durability_ros.py` - Launch file for easy startup
4. `README_ROS.md` - Comprehensive user documentation
5. `IMPLEMENTATION_SUMMARY.md` - This file

## Testing Checklist

- [ ] ATI sensor connects and publishes force data
- [ ] Camera initializes with correct settings
- [ ] CNC connects and can move
- [ ] Force thresholds trigger image capture correctly
- [ ] 0.25s delay enforced between CNC commands
- [ ] Images saved with correct naming (rising/falling)
- [ ] CSV log contains all data
- [ ] Repeat test completes 1000 cycles successfully
