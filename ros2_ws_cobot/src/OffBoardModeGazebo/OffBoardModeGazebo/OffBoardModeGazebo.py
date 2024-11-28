#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from enum import Enum

from px4_msgs.msg import VehicleCommand, OffboardControlMode, VehicleStatus


class State(Enum):
    WAITING_FOR_OFFBOARD = 1
    OFFBOARD_MODE = 2
    SENDING_HEARTBEAT = 3
    ATTEMPTING_ARM = 4


class QuadrotorOffboardArm(Node):
    def __init__(self):
        super().__init__("quadrotor_offboard_arm")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile,
        )

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )

        # Timer for checking the state at 2 Hz
        self.state_timer = self.create_timer(0.5, self.state_machine)

        self.state = State.WAITING_FOR_OFFBOARD
        self.offboard_mode = False  # Flag to indicate if offboard mode is active
        self.armed = False  # Flag to check if vehicle is armed
        self.updated = False
        self.last_state = None

    def arm_robot(self):
        vehicle_command = VehicleCommand()
        vehicle_command.timestamp = (
            self.get_clock().now().to_msg().sec * 1000000
        )  # PX4 expects timestamp in microseconds
        vehicle_command.param1 = 1.0  # 1 to arm, 0 to disarm
        vehicle_command.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        vehicle_command.target_system = 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 1
        vehicle_command.source_component = 1
        vehicle_command.from_external = True  # Indicate external command source

        self.publisher_vehicle_command.publish(vehicle_command)

    def disarm_robot(self):
        vehicle_command = VehicleCommand()
        vehicle_command.timestamp = (
            self.get_clock().now().to_msg().sec * 1000000
        )  # PX4 expects timestamp in microseconds
        vehicle_command.param1 = 0.0  # 1 to arm, 0 to disarm
        vehicle_command.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        vehicle_command.target_system = 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 1
        vehicle_command.source_component = 1
        vehicle_command.from_external = True  # Indicate external command source

        self.publisher_vehicle_command.publish(vehicle_command)

    def change_mode_to_offboard(self):
        """
        Change the mode of the vehicle to offboard mode

        This function sends a vehicle command to change the mode of the vehicle to offboard mode.

        Parameters:
            None

        Returns:
            None
        """
        if self.state != self.last_state:
            self.get_logger().warn("Changing mode to offboard")
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().to_msg().sec * 1000000
        msg.param1 = 1.0  # Main mode
        msg.param2 = 6.0  # Custom mode: Offboard
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.publisher_vehicle_command.publish(msg)

    def vehicle_status_callback(self, msg):
        # Process vehicle status message here
        self.updated = True
        self.offboard_mode = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.armed and not self.offboard_mode:
            self.get_logger().warn(
                "Disarming the vehicle as it is no longer in offboard mode"
            )
            self.disarm_robot()
        self.last_state = self.state

    def state_machine(self):
        if self.state == State.WAITING_FOR_OFFBOARD:
            if not self.offboard_mode:
                self.change_mode_to_offboard()
            else:
                self.state = State.OFFBOARD_MODE
                self.get_logger().info("Vehicle is now in Offboard mode")
        elif self.state == State.OFFBOARD_MODE:
            if self.offboard_mode:
                self.state = State.SENDING_HEARTBEAT
                self.get_logger().info("Starting to send Offboard heartbeat signal")
                self.heartbeat_signal_timer = self.create_timer(
                    0.2, self.send_offboard_control_mode
                )
                # Start timer to transition to ATTEMPTING_ARM state after 5 seconds
                self.create_timer(5.0, self.start_arm_attempt)
        elif self.state == State.SENDING_HEARTBEAT:
            # Vehicle is in offboard mode, continue sending control messages
            pass
        elif self.state == State.ATTEMPTING_ARM:
            if not self.armed:
                if self.state != self.last_state:
                    self.get_logger().warn("Attempting to arm the vehicle")
                self.arm_robot()
                # Retry arming every 2 seconds until successful
                self.arm_timer = self.create_timer(2.0, self.arm_robot)
                # Continue sending the heartbeat signal
                self.send_offboard_control_mode()
            else:
                if self.state != self.last_state:
                    self.get_logger().info("Vehicle is now armed")
        self.last_state = self.state

    def start_arm_attempt(self):
        if self.state == State.SENDING_HEARTBEAT:
            self.state = State.ATTEMPTING_ARM
            self.get_logger().info("Transitioning to ATTEMPTING_ARM state")

    def send_offboard_control_mode(self):
        ## Update the start time of the offboard message sender
        if not hasattr(self, "offboard_start_time"):
            self.offboard_start_time = self.get_clock().now()
        # Publish offboard control modes at 5 Hz
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.direct_actuator = True
        self.publisher_offboard_mode.publish(offboard_msg)


def main(args=None):
    rclpy.init(args=args)
    quadrotor_offboard_arm = QuadrotorOffboardArm()
    rclpy.spin(quadrotor_offboard_arm)
    quadrotor_offboard_arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
