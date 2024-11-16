#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from std_msgs.msg import Header
from actuator_msgs.msg import Actuators	 
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus, VehicleAttitude, VehicleLocalPosition, ActuatorMotors

def vector2PoseMsg(frame_id, position, attitude):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.orientation.w = attitude[0]
    pose_msg.pose.orientation.x = attitude[1]
    pose_msg.pose.orientation.y = attitude[2]
    pose_msg.pose.orientation.z = attitude[3]
    pose_msg.pose.position.x = float(position[0])
    pose_msg.pose.position.y = float(position[1])
    pose_msg.pose.position.z = float(position[2])
    return pose_msg

class QuadrotorMPC(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile)

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_rates_setpoint = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.publisher_actuator_cmd = self.create_publisher(Actuators, '/space_cobot0/motor/actuator_cmd', 10)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.offboard_started = False  # Flag to check if offboard was started
        self.arm_timer = None  # Timer for arming delay
        self.armed = False  # Flag to check if vehicle is armed

    def start_arm_timer(self):
        # Timer to arm the vehicle and switch to offboard mode 2 seconds after starting offboard control
        self.arm_timer = self.create_timer(2.0, self.arm_and_set_offboard_mode)

    def arm_and_set_offboard_mode(self):
        # Send arm command
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        print("Vehicle arming command sent")

        # Send offboard mode command
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # 1.0 for custom mode, 6.0 for offboard
        print("Offboard mode command sent")

        # Stop the timer after setting mode and arming
        self.arm_timer.cancel()

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Utility function to send a vehicle command."""
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.target_system = 1
        cmd.target_component = 1
        self.publisher_vehicle_command.publish(cmd)

    def vehicle_status_callback(self, msg):
        # Process vehicle status message here
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        print(f"NAV_STATUS: {self.nav_state}, ARMED: {self.armed}")

    def vehicle_attitude_callback(self, msg):
        # Process vehicle attitude here
        pass

    def vehicle_local_position_callback(self, msg):
        # Process local position here
        pass

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.direct_actuator = True
        self.publisher_offboard_mode.publish(offboard_msg)

        if not self.offboard_started:
            # Start the arm and offboard mode timer on the first offboard message
            self.start_arm_timer()
            self.offboard_started = True

        if self.armed:
            # Only publish actuator commands if the vehicle is armed
            actuator_cmd_msg = Actuators()
            # Fill in the Header
            actuator_cmd_msg.header = Header()
            actuator_cmd_msg.header.stamp = self.get_clock().now().to_msg()  # get current time
            actuator_cmd_msg.header.frame_id = "base_link"
            actuator_cmd_msg.velocity = [100.0] * 6
            self.publisher_actuator_cmd.publish(actuator_cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    quadrotor_mpc = QuadrotorMPC()
    rclpy.spin(quadrotor_mpc)
    quadrotor_mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
