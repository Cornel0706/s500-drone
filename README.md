# S500 Autonomous SLAM - Real Drone Visual SLAM & Exploration

This package enables autonomous Visual SLAM exploration on a real HoneyBro S500 V2 drone. It uses an RGBD camera for mapping and a custom MRTSP algorithm for frontier-based exploration.

## Hardware Requirements
- Drone Frame: HoneyBro S500 V2
- Flight Controller: Pixhawk 6C Mini
- Companion Computer: Raspberry Pi 5
- Sensors: RGBD Camera (e.g. Intel RealSense D435i or OAK-D)
- Power: UBEC (5V for Raspberry Pi 5)

## Software Prerequisites
- ROS 2 Jazzy
- PX4 Autopilot
- px4_msgs (Micro-XRCE-DDS)
- RTAB-Map
- Nav2
- MRTSP (Frontier Exploration)

## Hardware Setup Instructions

1. **UART Connection**: Connect the RPi5 UART to the Pixhawk 6C Mini TELEM port.
2. **UBEC Power**: Connect the UBEC to power the Raspberry Pi 5 from the drone battery.
3. **PX4 Parameters** (via QGroundControl):
   - Set `UXRCE_DDS_CFG` to the TELEM port you connected to.
   - Set `UXRCE_DDS_BAUD` to `921600` (or matching baud rate).

## RPi5 UART Setup
On the Raspberry Pi 5, enable UART and disable the serial console:
1. Edit `/boot/firmware/config.txt`.
2. Add `enable_uart=1`.
3. Disable the serial console from `/boot/firmware/cmdline.txt` by removing `console=serial0,115200`.

## Build Instructions
```bash
mkdir -p ~/ros2_ws_frontier_detection/src
cd ~/ros2_ws_frontier_detection/src
git clone <repository_url> .
cd ~/ros2_ws_frontier_detection
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Autonomous Exploration
Run the autonomous SLAM and exploration package:
```bash
ros2 launch s500_autonomous_slam real_drone_slam.launch.py
```

### Manual Teleoperation (Teleop)
For manual flight while running SLAM:
```bash
ros2 launch s500_autonomous_slam real_drone_teleop.launch.py
```

## Testing Procedure
Follow these 5 phases to safely test the system:
1. **Bench Test**: Test communication between RPi5 and Pixhawk without props.
2. **Sensor Test**: Verify camera data and RTAB-Map locally on RPi5.
3. **Manual Flight Test**: Use teleop in a safe area to check offboard control.
4. **Semi-Autonomous Test**: Use Nav2 to send simple goals via RViz.
5. **Full Autonomous Test**: Run MRTSP for full exploration.

## Camera Setup
*(Placeholder for camera-specific instructions)*
- RealSense: `sudo apt install ros-jazzy-realsense2-camera`
- OAK-D: `sudo apt install ros-jazzy-depthai-ros`

## Safety Warnings
- ALWAYS fly in an open area with a safety pilot ready to take manual control.
- Verify GPS lock or reliable Visual Odometry before switching to Offboard mode.
- Use a tether or prop guards for initial tests.

## Troubleshooting
- **No Micro-XRCE-DDS Connection**: Double-check UART wiring (TX to RX, RX to TX) and baud rate in QGroundControl.
- **Drone Drifts in Offboard**: Check RTAB-Map output and odometry drift in RViz.
- **Nav2 Failures**: Ensure costmaps are updating correctly and inflation radius is large enough.
