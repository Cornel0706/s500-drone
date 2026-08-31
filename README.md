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


## Camera Setup
*(Placeholder for camera-specific instructions)*
- OAK-D: `sudo apt install ros-jazzy-depthai-ros`

## Troubleshooting
- **No Micro-XRCE-DDS Connection**: Double-check UART wiring (TX to RX, RX to TX) and baud rate in QGroundControl.
- **Drone Drifts in Offboard**: Check RTAB-Map output and odometry drift in RViz.
- **Nav2 Failures**: Ensure costmaps are updating correctly and inflation radius is large enough.

## 🌐 Network & SSH Headless Setup (Wi-Fi Hotspot)

To control the robot/drone in the field without an external router or monitor, you can configure the Raspberry Pi (or any Linux companion computer) to broadcast its own Wi-Fi network. This is crucial for outdoor autonomous flights.

### 1. Enable SSH
Install and enable the SSH server so you can connect remotely:
```bash
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
```

### 2. Create the Wi-Fi Hotspot
Run this command to create a network named `Robot_Hotspot` with the password `12345678`. The device will automatically be assigned the IP `10.42.0.1`.
```bash
nmcli device wifi hotspot ifname wlan0 ssid Robot_Hotspot password "12345678"
```
To ensure the hotspot always starts automatically on boot (even if other known networks are present), give it maximum priority:
```bash
nmcli connection modify Hotspot connection.autoconnect yes connection.autoconnect-priority 100
```

### 3. Connect from Laptop (Ground Control Station)
1. Connect your laptop to the `Robot_Hotspot` Wi-Fi network.
2. Open a terminal and connect via SSH:
   ```bash
   ssh ubuntu@10.42.0.1
   ```
   *(Replace `ubuntu` with your actual Linux username).*

**Passwordless SSH (Optional but recommended):**
To avoid typing the password every time, run these commands from your laptop terminal (while disconnected from SSH):
```bash
ssh-keygen -t ed25519
ssh-copy-id ubuntu@10.42.0.1
```

### 4. Switch Back to Internet
To download updates or packages, you can temporarily disable the hotspot and reconnect to your home router:
```bash
nmcli connection down Hotspot
```
When heading back to the field, simply re-enable it:
```bash
nmcli connection up Hotspot
```

