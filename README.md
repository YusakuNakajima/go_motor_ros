# Go Motor ROS Package

Independent ROS package for controlling Unitree GO motors with dynamic parameter tuning and topic-based control. 
This package includes the complete Unitree Motor SDK and does not depend on `go_motor_control`.

## Unitree Actuator SDK

This package includes a built version of the Unitree Actuator SDK from 2025/07/25 (https://github.com/unitreerobotics/unitree_actuator_sdk) to reduce external dependencies. The SDK is built and included within this repository. If needed, please update to the latest version from the official repository.

## Tested Hardware

This package has been tested with **GO-M8010-6** motors. Support for A1 and B1 motors is included in the code but has not been verified through testing.

## Features

- **Automatic Control Mode Selection**: Automatically selects optimal control parameters based on active commands
- **Safety Timeout Protection**: Automatically stops motor if no commands received within timeout period  
- **Dynamic Reconfigure**: Adjust `kp`, `kd`, and `id` parameters in real-time using `rqt_reconfigure`
- **ROS Parameter Server**: Initialize parameters from launch files or YAML config files
- **Topic-based Control**: Send position, velocity, and torque commands via ROS topics
- **Joint State Publishing**: Monitor motor state through standard `sensor_msgs/JointState`
- **C++ Implementation**: Real-time motor control with Unitree Motor SDK
- **Independent**: Complete SDK included, no external dependencies

## Quick Start

### 1. Launch the Motor Controller
```bash
roslaunch go_motor_ros go_motor_ros.launch
```

### 2. Make the Motor Rotate Slowly (Velocity Control)
```bash
# Send velocity command to make motor rotate slowly at 1 rad/s
rostopic pub /go_motor_ros/motor_command/velocity std_msgs/Float64 "data: 1.0"
```

### 3. Stop the Motor
```bash
# Stop motor by setting velocity to 0
rostopic pub /go_motor_ros/motor_command/velocity std_msgs/Float64 "data: 0.0"
```

### 4. Adjust Parameters (Optional)
Use `rqt_reconfigure` (automatically launched) to adjust:
- **id**: Motor ID (0-15)
- **kp**: Set to 0.0 for velocity control
- **kd**: Damping gain (0.1 recommended for smooth motion)

## Usage

### Basic Launch

```bash
# Basic launch for real motor (loads default_params.yaml)
roslaunch go_motor_ros go_motor_ros.launch

# With custom config file
roslaunch go_motor_ros go_motor_ros.launch config_file:=$(find go_motor_ros)/config/custom_params.yaml

# Without GUI tools
roslaunch go_motor_ros go_motor_ros.launch launch_gui_tools:=false
```

### Dynamic Parameter Tuning

Use `rqt_reconfigure` to adjust parameters in real-time:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

### Sending Motor Commands

#### Using rqt_publisher (GUI)
The launch files automatically start `rqt_publisher` for easy command sending through a graphical interface.

#### Using rostopic (Command Line)

#### Position Control Mode
Simply send position commands - control parameters are automatically configured:
```bash
# Send position command (automatic: kp=10.0, kd=1.0)
rostopic pub /go_motor_ros/motor_command/position std_msgs/Float64 "data: 1.57"

# Move to zero position
rostopic pub /go_motor_ros/motor_command/position std_msgs/Float64 "data: 0.0"
```

#### Velocity Control Mode  
Simply send velocity commands - control parameters are automatically configured:
```bash
# Send velocity command (automatic: kp=0.0, kd=1.0)
rostopic pub /go_motor_ros/motor_command/velocity std_msgs/Float64 "data: 2.0"

# Stop motor
rostopic pub /go_motor_ros/motor_command/velocity std_msgs/Float64 "data: 0.0"
```

#### Torque Control Mode
Simply send torque commands - control parameters are automatically configured:
```bash
# Send torque command (automatic: kp=0.0, kd=0.0)
rostopic pub /go_motor_ros/motor_command/torque std_msgs/Float64 "data: 0.5"
```

#### Mixed Control Mode
When multiple command types are active, the system uses priority: Position > Velocity > Torque.
Commands can be combined and the appropriate control gains are automatically selected.

### Monitor Motor State

```bash
rostopic echo /go_motor_ros/motor_state
```

## Parameters

### ROS Parameters (set via launch file or rosparam)
- `motor_type` (string): Type of Unitree motor (default: "GO_M8010_6")
  - Supported types: "A1", "B1", "GO_M8010_6"
- `serial_port` (string): Serial port for communication (default: "/dev/ttyUSB0")
  - Example: "/dev/ttyUSB0" or "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9HN1N7-if00-port0"
- `control_frequency` (double): Control loop frequency in Hz (default: 200.0)
- `command_timeout` (double): Safety timeout for motor commands in seconds (default: 1.0)

### Dynamic Reconfigure Parameters
- `id` (int): Motor ID (0-15, default: 0)
- `kp` (double): Proportional gain for position control (0.0-100.0, default: 0.0)
- `kd` (double): Derivative gain for damping control (0.0-50.0, default: 0.01)

## Topics

### Subscribed Topics
- `/go_motor_ros/motor_command/position` (std_msgs/Float64): Target position in radians
- `/go_motor_ros/motor_command/velocity` (std_msgs/Float64): Target velocity in rad/s
- `/go_motor_ros/motor_command/torque` (std_msgs/Float64): Target torque in Nm

### Published Topics
- `/go_motor_ros/motor_state` (sensor_msgs/JointState): Current motor state

## Configuration Files

- `config/default_params.yaml`: Default parameter values
- `cfg/MotorParams.cfg`: Dynamic reconfigure configuration

## Documentation

Additional documentation and manuals are available in `catkin_ws/src/go_motor_ros/doc/`. Please refer to these documents for detailed information about motor setup, calibration, and advanced usage.

### Launch Arguments

- `motor_type`: Motor type (default: "GO_M8010_6")
- `serial_port`: Serial port path (default: "/dev/ttyUSB0")  
- `control_frequency`: Control loop frequency in Hz (default: 200.0)
- `command_timeout`: Safety timeout in seconds (default: 1.0)
- `config_file`: Path to YAML configuration file
- `launch_gui_tools`: Whether to launch rqt_reconfigure and rqt_publisher (default: true)

## Automatic Control Mode Selection

The system automatically selects the appropriate control mode and parameters based on which commands are actively being sent:

### Control Mode Priority
When multiple command types are active simultaneously, the system uses the following priority:
1. **Position Control** (highest priority)
2. **Velocity Control** 
3. **Torque Control** (lowest priority)

### Automatic Parameter Configuration

| Control Mode | Conditions | Auto-Set Parameters | Description |
|--------------|------------|-------------------|-------------|
| **Position Control** | Position commands active | `kp = configured value`<br>`kd = configured value` | Full PD control for precise positioning |
| **Velocity Control** | Velocity commands active<br>(no position commands) | `kp = 0.0`<br>`kd = configured value` | Pure velocity control with damping |
| **Torque Control** | Torque commands active<br>(no position/velocity commands) | `kp = 0.0`<br>`kd = 0.0` | Direct torque control |
| **Damping Mode** | No active commands<br>(all timed out) | `kp = 0.0`<br>`kd = configured value` | Free motion with damping |

### Example Usage
```bash
# Position control - automatically sets kp=10.0, kd=1.0
rostopic pub /go_motor_ros/motor_command/position std_msgs/Float64 "data: 1.57"

# Switch to velocity control - automatically sets kp=0.0, kd=1.0  
rostopic pub /go_motor_ros/motor_command/velocity std_msgs/Float64 "data: 2.0"

# The system automatically switches modes based on the most recent command type
```

## Safety Timeout Protection

The motor controller includes built-in safety features to prevent runaway conditions:

### Timeout Behavior
- **Default timeout**: 1.0 second (configurable via `command_timeout` parameter)
- **Individual monitoring**: Each command type (position, velocity, torque) is monitored separately
- **Automatic stop**: Motor stops when ALL command types have timed out
- **Automatic restart**: Motor resumes operation when any new command is received

### Safety Actions
1. **Normal Operation**: Commands received within timeout → Normal control with appropriate mode
2. **Partial Timeout**: Some commands timed out → Timed out commands set to zero, others continue
3. **Full Timeout**: All commands timed out → Emergency stop (zero torque mode: `kp=0, kd=0`)
4. **Recovery**: New command received → Automatic restart with appropriate control mode

### Configuration Examples
```bash
# Short timeout for high-safety applications (0.5 seconds)
roslaunch go_motor_ros go_motor_ros.launch command_timeout:=0.5

# Longer timeout for slower control systems (3.0 seconds)  
roslaunch go_motor_ros go_motor_ros.launch command_timeout:=3.0

# Disable GUI tools for production use
roslaunch go_motor_ros go_motor_ros.launch launch_gui_tools:=false command_timeout:=1.0
```

### Monitoring Timeout Status
The system provides detailed logging of timeout status:
```
[INFO] Control Mode - pos_cmd=1.570(timeout=NO), vel_cmd=0.000(timeout=YES), torque_cmd=0.000(timeout=YES)
[WARN] Motor command timeout (1.0 sec) - stopping motor for safety
[INFO] Motor restarted by position command: 1.570
```

## Implementation Details

### C++ Node
- Uses actual Unitree Motor SDK for real motor communication
- Includes error handling for serial port failures (falls back to simulation)
- Applies proper gear ratio calculations for A1/B1/GO_M8010_6 motors
- High-frequency control loop (up to 1000Hz)
- Real-time parameter updates via dynamic reconfigure

### Logging Output
The node provides detailed logging every second (at 1000Hz control frequency) showing both command and received values:
```
Motor 0 - CMD: pos=1.570, vel=2.000, torque=0.500, kp=10.000, kd=1.000 | RECV: pos=1.572, vel=1.998, torque=0.498
```

- **CMD**: Values sent to the motor (position, velocity, torque, kp, kd)
- **RECV**: Values received from the motor (actual position, velocity, torque)

### Error Handling
- Serial port connection failures are handled gracefully
- Motor communication errors are logged but don't crash the node
- Invalid motor types default to GO_M8010_6 with warnings

## Troubleshooting

### Serial Port Issues
1. **Permission denied**: Add user to dialout group
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log in again
   ```

2. **Port not found**: Check available ports
   ```bash
   ls /dev/ttyUSB*
   ls /dev/serial/by-id/
   ```

3. **Motor not responding**: Check connections and motor power

### Common Error Messages
- `Failed to open serial port`: Check port permissions and cable connection
- `Unknown motor type`: Verify motor_type parameter in config file
- `Running in simulation mode`: Serial port failed, check hardware connection

### Control Mode Issues
1. **Position not working**: Check that position commands are being received (not timed out)
2. **Velocity control not working**: Ensure no position commands are active (they have higher priority)
3. **Motor stops unexpectedly**: Check timeout settings - may be too short for your application
4. **Wrong control mode selected**: Check command timeout status in logs
5. **Monitor control mode**: Look for `Using Position Control Mode` messages in debug logs

### Debugging
1. Enable debug messages:
   ```bash
   rosservice call /go_motor_ros/go_motor_ros_node/set_logger_level ros.go_motor_ros DEBUG
   ```

2. Monitor topics in real-time:
   ```bash
   rostopic hz /go_motor_ros/motor_state
   rostopic echo /go_motor_ros/motor_state
   ```

## Dependencies

- roscpp
- rospy  
- std_msgs
- geometry_msgs
- sensor_msgs
- dynamic_reconfigure
- Unitree Motor SDK (included)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.