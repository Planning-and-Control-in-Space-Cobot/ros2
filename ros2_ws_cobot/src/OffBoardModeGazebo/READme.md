# Quadrotor MPC Node

This ROS 2 node, `QuadrotorMPC`, is designed to control a quadrotor using Model Predictive Control (MPC) in offboard mode. It integrates with the PX4 Autopilot framework and publishes commands directly to the vehicle's actuators.

## Features

- **Subscription to vehicle data topics:**
  - Vehicle status (`/fmu/out/vehicle_status`)
  - Vehicle attitude (`/fmu/out/vehicle_attitude`)
  - Local position (`/fmu/out/vehicle_local_position`)
- **Command publishing:**
  - Offboard control mode (`/fmu/in/offboard_control_mode`)
  - Actuator motor rates (`/fmu/in/actuator_motors`)
  - Vehicle commands (`/fmu/in/vehicle_command`)
  - Custom actuator commands (`/space_cobot0/motor/actuator_cmd`)
- **Offboard mode management:**
  - Automatic arming and offboard mode activation
  - Periodic actuator command publishing when the vehicle is armed
- **High-frequency command loop (50 Hz)** for consistent control updates.

## Dependencies

This node relies on the following ROS 2 message types and PX4-specific topics:

- `nav_msgs/Path`
- `geometry_msgs/PoseStamped`
- `visualization_msgs/Marker`
- `std_msgs/Header`
- `actuator_msgs/Actuators`
- `px4_msgs` (e.g., `OffboardControlMode`, `VehicleCommand`, `VehicleStatus`, `VehicleAttitude`, `VehicleLocalPosition`, `ActuatorMotors`)

## Code Overview

### Key Functions

#### `vector2PoseMsg(frame_id, position, attitude)`

Utility function to convert position and attitude data into a `PoseStamped` message.

#### `QuadrotorMPC`

The main node class for managing subscriptions, publishing commands, and handling offboard mode operations.

- **Initialization:**
  - Sets up publishers and subscribers.
  - Initializes timers for command loops and arming/offboard mode delays.
- **Callbacks:**
  - `vehicle_status_callback`: Monitors arming state and navigation state.
  - `vehicle_attitude_callback`: Placeholder for attitude processing.
  - `vehicle_local_position_callback`: Placeholder for position processing.
- **Command loop:**
  - Publishes the offboard control mode and actuator commands periodically.
  - Handles arming and mode transitions.

### Usage Flow

1. Start the node.
2. The node waits for vehicle status updates.
3. On the first offboard control message, a timer is initiated to:
   - Arm the vehicle.
   - Set it to offboard mode.
4. When the vehicle is armed, actuator commands are periodically published to control the motors.

## Configuration

The QoS (Quality of Service) settings are configured for reliable communication:

- **Reliability**: `BEST_EFFORT`
- **Durability**: `TRANSIENT_LOCAL`
- **History**: `KEEP_LAST`

## How to Run

### Prerequisites

1. Ensure ROS 2 and PX4 are installed and properly configured.
2. The PX4 SITL or hardware setup should be ready to accept offboard commands.

### Execution

Run the node with:

```bash
ros2 run <your_package_name> quadrotor_mpc

