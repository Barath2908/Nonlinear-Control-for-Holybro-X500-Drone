# Holybro X500 GPS-Denied Indoor Autonomy Stack

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-4.5+-orange)](https://ardupilot.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A production-ready ROS 2 package for GPS-denied indoor autonomous flight on Holybro X500 quadcopter. This system feeds Visual-Inertial Odometry (VIO) into ArduPilot's EKF3 via a custom MAVLink external-vision bridge with sliding mode control.

## Performance Metrics

| Metric | Value |
|--------|-------|
| End-to-end Vision Latency | 85 ms |
| EKF3 Convergence Time | 14 s |
| RMS XY Drift (90s hover) | 0.45 m |
| RMS Altitude Error | 0.18 m |

## System Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   OAK-D/T265    │────▶│  VIO Pipeline    │────▶│  ROS 2 Bridge   │
│   Stereo Cam    │     │  (DepthAI/T265)  │     │                 │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
                                                          │
                                                          ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Holybro X500  │◀────│  Pixhawk 6X      │◀────│  MAVLink Bridge │
│   Airframe      │     │  ArduPilot EKF3  │     │  VISION_POSITION│
└─────────────────┘     └──────────────────┘     └─────────────────┘
                                                          │
                                                          ▼
                                               ┌─────────────────────┐
                                               │  Sliding Mode Ctrl  │
                                               │  Position Control   │
                                               └─────────────────────┘
```

## Hardware Requirements

- **Flight Controller**: Pixhawk 6X (or compatible)
- **Companion Computer**: Raspberry Pi 5 (8GB recommended) / Jetson Nano
- **VIO Sensor**: Intel RealSense T265 or OAK-D Pro
- **Frame**: Holybro X500 V2
- **Communication**: USB-C (companion to FC)

## Software Dependencies

- Ubuntu 22.04 LTS
- ROS 2 Humble
- ArduPilot 4.5+ (Copter)
- MAVSDK or pymavlink
- depthai-ros (for OAK-D) or realsense2_camera (for T265)

## Quick Start

### 1. Clone and Build

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/yourusername/holybro_x500_autonomy.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select vio_mavlink_bridge
source install/setup.bash
```

### 2. Configure ArduPilot Parameters

Load the provided parameter file or set manually:

```bash
# Connect to vehicle via MAVProxy
mavproxy.py --master=/dev/ttyACM0 --baudrate=921600

# Load parameters
param load config/ardupilot_params.param
```

**Critical EKF3 Parameters:**
```
EK3_SRC1_POSXY = 6    # ExternalNav
EK3_SRC1_VELXY = 6    # ExternalNav  
EK3_SRC1_POSZ = 1     # Baro (or 6 for ExternalNav)
EK3_SRC1_VELZ = 6     # ExternalNav
EK3_SRC1_YAW = 6      # ExternalNav
VISO_TYPE = 1         # MAVLink
VISO_DELAY_MS = 50    # Adjust based on your latency
```

### 3. Calibrate Stereo Cameras

```bash
# Run P1/P2 stereo calibration
ros2 run vio_mavlink_bridge stereo_calibration.py \
    --board-size 9x6 \
    --square-size 0.025 \
    --output-file config/stereo_calib.yaml
```

### 4. Launch the System

```bash
# Terminal 1: Launch VIO
ros2 launch vio_mavlink_bridge vio.launch.py

# Terminal 2: Launch MAVLink bridge
ros2 launch vio_mavlink_bridge mavlink_bridge.launch.py

# Terminal 3: (Optional) Launch sliding mode controller
ros2 launch vio_mavlink_bridge sliding_mode_ctrl.launch.py
```

### 5. Production Deployment with systemd

```bash
# Install systemd services
sudo cp systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable vio-mavlink-bridge.service
sudo systemctl start vio-mavlink-bridge.service
```

## Package Structure

```
holybro_x500_autonomy/
├── README.md
├── LICENSE
├── src/
│   └── vio_mavlink_bridge/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/
│       │   └── vio_mavlink_bridge/
│       │       ├── mavlink_bridge_node.hpp
│       │       ├── sliding_mode_controller.hpp
│       │       └── vio_processor.hpp
│       ├── src/
│       │   ├── mavlink_bridge_node.cpp
│       │   ├── sliding_mode_controller.cpp
│       │   └── vio_processor.cpp
│       ├── launch/
│       │   ├── vio.launch.py
│       │   ├── mavlink_bridge.launch.py
│       │   └── full_stack.launch.py
│       ├── config/
│       │   ├── bridge_params.yaml
│       │   ├── controller_params.yaml
│       │   └── ardupilot_params.param
│       └── scripts/
│           ├── stereo_calibration.py
│           └── ekf_monitor.py
├── systemd/
│   ├── vio-mavlink-bridge.service
│   └── vio-autonomy.service
├── calibration/
│   └── stereo_calib_example.yaml
├── docs/
│   ├── CALIBRATION.md
│   ├── TROUBLESHOOTING.md
│   └── TUNING.md
└── test/
    └── test_mavlink_bridge.cpp
```

## Configuration

### Bridge Parameters (`config/bridge_params.yaml`)

```yaml
mavlink_bridge:
  ros__parameters:
    serial_port: "/dev/ttyACM0"
    baudrate: 921600
    vio_topic: "/camera/odom/sample"
    frame_id: "odom"
    child_frame_id: "base_link"
    publish_rate: 30.0
    vision_delay_ms: 50
    coordinate_frame: 21  # MAV_FRAME_LOCAL_FRD
```

### Sliding Mode Controller (`config/controller_params.yaml`)

```yaml
sliding_mode_controller:
  ros__parameters:
    # Sliding surface parameters
    lambda_xy: 2.0
    lambda_z: 1.5
    
    # Control gains
    k_xy: 5.0
    k_z: 3.0
    eta: 0.1  # Boundary layer thickness
    
    # Limits
    max_velocity_xy: 2.0
    max_velocity_z: 1.0
    max_acceleration: 3.0
```

## Coordinate Frame Conventions

This package follows **NED (North-East-Down)** convention for ArduPilot compatibility:

```
ArduPilot NED          Camera (typical)
    X (North)              Z (forward)
    │                      │
    │                      │
    └──── Y (East)         └──── X (right)
    
    Z points Down          Y points Down
```

The bridge automatically transforms VIO poses from camera frame to NED.

## Troubleshooting

### EKF3 Not Converging

1. Check `VISO_DELAY_MS` matches actual latency
2. Verify VIO topic is publishing at expected rate
3. Ensure proper camera calibration
4. Check MAVLink `VISION_POSITION_ESTIMATE` messages in MAVProxy

```bash
# Monitor in MAVProxy
module load message
message VISION_POSITION_ESTIMATE
```

### High Drift

1. Re-run stereo calibration with better lighting
2. Check for vibrations affecting IMU
3. Verify time synchronization between companion computer and FC
4. Reduce `publish_rate` if CPU is overloaded

### Latency Issues

1. Use USB 3.0 for camera connection
2. Reduce VIO resolution if needed
3. Check CPU governor is set to `performance`
4. Monitor with: `ros2 topic hz /camera/odom/sample`

## Safety Considerations

⚠️ **WARNING**: Always test in a controlled environment with safety nets first.

1. **Pre-flight checks**:
   - Verify EKF3 convergence before arming
   - Check VIO health via `/vio/status` topic
   - Ensure adequate lighting for visual features

2. **Failsafe configuration**:
   - Set `FS_EKF_ACTION` to land or RTL
   - Configure `EK3_ERR_THRESH` appropriately
   - Test failsafe behavior before autonomous flight

3. **Recommended first flight**:
   - Start in LOITER mode with GPS available
   - Gradually reduce GPS influence
   - Monitor EKF innovation values

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit changes (`git commit -am 'Add new feature'`)
4. Push to branch (`git push origin feature/improvement`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file.

## Acknowledgments

- ArduPilot Development Team
- ROS 2 Community
- Intel RealSense Team
- Luxonis (OAK-D)

## Citation

If you use this work in your research, please cite:

```bibtex
@software{holybro_x500_autonomy,
  author = {Barath Kumar JK},
  title = {GPS-Denied Indoor Autonomy for Holybro X500},
  year = {2025},
  url = {https://github.com/yourusername/holybro_x500_autonomy}
}
```

## Contact

- **Author**: Barath Kumar JK
- **Email**: bj2519@columbia.edu
- **LinkedIn**: [linkedin.com/in/jkb2](https://linkedin.com/in/jkb2)
