import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSPresetProfiles,
)
from ament_index_python.packages import get_package_share_directory

## Ros2 message modules

# PX4 messages
from px4_msgs.msg import (
    SensorCombined,
    VehicleAngularVelocity,
    VehicleAttitude,
    VehicleLocalPosition,
)

# STD messages
from std_msgs.msg import Float64MultiArray

# Actuators messages
from actuator_msgs.msg import Actuators

from full_system_controller.controller import FullSystemDynamics as MPC
from full_system_controller.ThrustToRpm import ThrustToRpm, MotorType

import os
import time
import numpy as np
import sympy as sp

import scipy.spatial.transform as trf

from typing import List, Dict


class ControllerNode(Node):
    def __init__(self):
        super().__init__("full_system_controller")
        self.sol = None

        ## Load parameters
        self.declare_parameter("horizon", 10)
        self.declare_parameter("horizon_dt", 0.1)
        self.declare_parameter("controller_dt", 0.1)  ## Controller update rate
        self.declare_parameter("mass", 3.4)
        self.declare_parameter("desired_attitude", [20.0, 0.0, 0.0])
        self.declare_parameter("desired_position", [1.0, 0.0, 0.0])
        self.declare_parameter("J_matrix", "no_file_path")
        self.declare_parameter("c_matrix", "no_file_path")
        self.declare_parameter("A_matrix", "no_file_path")
        self.declare_parameter("thrust_model", "no_file_path")

        horizon = self.get_parameter("horizon").get_parameter_value().integer_value
        horizon_dt = self.get_parameter("horizon_dt").get_parameter_value().double_value
        controller_dt = (
            self.get_parameter("controller_dt").get_parameter_value().double_value
        )
        mass = self.get_parameter("mass").get_parameter_value().double_value
        desired_attitude = (
            self.get_parameter("desired_attitude")
            .get_parameter_value()
            .double_array_value
        )

        self.desired_attitude = trf.Rotation.from_euler(
            "xyz", desired_attitude, degrees=True
        )

        desired_position = (
            self.get_parameter("desired_position")
            .get_parameter_value()
            .double_array_value
        )
        J_matrix = self.get_parameter("J_matrix").get_parameter_value().string_value
        c_matrix = self.get_parameter("c_matrix").get_parameter_value().string_value
        A_matrix = self.get_parameter("A_matrix").get_parameter_value().string_value
        thrust_model = (
            self.get_parameter("thrust_model").get_parameter_value().string_value
        )

        package_name = "full_system_controller"
        package_share_directory = get_package_share_directory(package_name)
        if J_matrix == "no_file_path":
            self.get_logger().warn("Using the default J matrix")
            J_matrix = os.path.join(package_share_directory, "configs", "J_matrix.npy")

        if c_matrix == "no_file_path":
            self.get_logger().warn("Using the default c matrix")
            c_matrix = os.path.join(package_share_directory, "configs", "c_vector.npy")

        if A_matrix == "no_file_path":
            self.get_logger().warn("Using the default A matrix")
            A_matrix = os.path.join(package_share_directory, "configs", "A_matrix.npy")

        if thrust_model == "no_file_path":
            self.get_logger().warn("Using the default thrust to rpm model")
            thrust_model = os.path.join(
                package_share_directory, "configs", "thrust_model.npy"
            )

        self.desired_position = np.array(desired_position)

        J = np.load(J_matrix)
        c = np.load(c_matrix)
        A = np.load(A_matrix)
        thrust_to_rpm = np.load(thrust_model)
        max_thrust = 2.0
        min_thrust = -2.0

        self.Thrust_to_rpm_converter = ThrustToRpm(
            thrust_to_rpm, min_thrust, max_thrust
        )

        self.mpc = MPC(J, c, A, mass, horizon, horizon_dt)

        solver_options = {
            "ipopt": {
                "print_level": 0,
                "tol": 1e-2,
                "acceptable_tol": 1e-2,
                "acceptable_iter": 5,
                "linear_solver": "mumps",
                "mu_strategy": "adaptive",
                "hessian_approximation": "limited-memory",
                "warm_start_init_point": "yes",
                "warm_start_bound_push": 1e-6,
                "warm_start_bound_frac": 1e-6,
                "max_cpu_time": 1.5,
            },
            "print_time": 0,
            "jit": True,
            "jit_cleanup": True,
            "jit_options": {
                "flags": "",
            },
        }

        self.mpc.setup_problem_no_reference(solver_options)

        ## This call is to compile the JIT before even starting actuation
        sol = self.mpc.solve_problem_no_reference(
            np.zeros(3),
            np.zeros(3),
            trf.Rotation.from_euler("xyz", [0.0, 0.0, 0.0], degrees=True),
            np.zeros(3),
            np.zeros(3),
            np.zeros(3),
            trf.Rotation.from_euler("xyz", [20.0, 0.0, 0.0], degrees=True),
            np.array([-1.0, -1.0, -2.0]),
        )

        ## save data
        # np.save("u.npy", sol.value(self.mpc.u))
        # np.save("w_dot.npy", sol.value(self.mpc.w_dot))
        # np.save("w.npy", sol.value(self.mpc.w))
        # np.save("q.npy", sol.value(self.mpc.q))
        # np.save("v_dot.npy", sol.value(self.mpc.v_dot))
        # np.save("v.npy", sol.value(self.mpc.v))
        # np.save("p.npy", sol.value(self.mpc.p))
        # np.save("Moment.npy", sol.value(self.mpc.Moment))
        # np.save("F.npy", sol.value(self.mpc.F))

        ## Declare subscribers
        self.sensors_combined_subscriber = self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self.sensors_combined_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.vehicle_angular_velocity_subscriber = self.create_subscription(
            VehicleAngularVelocity,
            "/fmu/out/vehicle_angular_velocity_groundtruth",
            self.vehicle_angular_velocity_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude_groundtruth",
            self.vehicle_attitude_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.vehicle_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_groundtruth",
            self.vehicle_local_position_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        ## Declare the publisher
        self.actuation_publisher = self.create_publisher(
            Actuators, "/space_cobot0/motor/actuator_cmd", 10
        )

        self.controller_timer = self.create_timer(
            controller_dt, self.controller_timer_callback
        )

        self.w_debug_pub = self.create_publisher(Float64MultiArray, "/fmu/debug/w", 10)

        self.w_dot_debug_pub = self.create_publisher(
            Float64MultiArray, "/fmu/debug/w_dot", 10
        )

        self.q_debug_pub = self.create_publisher(Float64MultiArray, "/fmu/debug/q", 10)

        self.v_debug_pub = self.create_publisher(Float64MultiArray, "/fmu/debug/v", 10)

        self.v_dot_debug_pub = self.create_publisher(
            Float64MultiArray, "/fmu/debug/v_dot", 10
        )

        self.p_debug_pub = self.create_publisher(Float64MultiArray, "/fmu/debug/p", 10)

    def controller_timer_callback(self):
        if not hasattr(self, "vehicle_pos"):
            self.get_logger().warn(
                "messages from local position topic are yet to arrive"
            )
        elif not hasattr(self, "vehicle_attitude"):
            self.get_logger().warn("message from attitude callback are yet to arrive")
        elif not hasattr(self, "vehicle_angular_speed"):
            self.get_logger().warn(
                "message from angular velocity callback are yet to arrive"
            )
        else:
            u_vector = self.mpc.solve_problem_no_reference(
                self.vehicle_angular_acceleration,
                self.vehicle_angular_speed,
                self.vehicle_attitude,
                self.vehicle_acceleration,
                self.vehicle_velocity,
                self.vehicle_pos,
                self.desired_attitude,
                self.desired_position,
            )

            # print("Time to get u_vector: ", time.time() - time_start)
            rpm = []
            print("u_vector: ", u_vector)

            for u in u_vector:
                rpm.append(self.Thrust_to_rpm_converter.get_rpm(u))

            msg = Actuators()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.velocity = rpm

            self.actuation_publisher.publish(msg)

            # w_ = self.sol_.value(self.mpc.w)[:, :]
            # w_dot_ = self.sol_.value(self.mpc.w_dot)[:, :]
            # q_ = self.sol_.value(self.mpc.q)[:, :]
            # v_ = self.sol_.value(self.mpc.v)[:, :]
            # v_dot_ = self.sol_.value(self.mpc.v_dot)[:, :]
            # p_ = self.sol_.value(self.mpc.p)[:, :]

            # w_msg = Float64MultiArray()
            # w_msg.data = w_.astype(float).flatten().tolist()
            # self.w_debug_pub.publish(w_msg)

            # w_dot_msg = Float64MultiArray()
            # w_dot_msg.data = w_dot_.astype(float).flatten().tolist()

            # self.w_dot_debug_pub.publish(w_dot_msg)

            # q_msg = Float64MultiArray()
            # q_msg.data = q_.astype(float).flatten().tolist()
            # self.q_debug_pub.publish(q_msg)

            # v_msg = Float64MultiArray()
            # v_msg.data = v_.astype(float).flatten().tolist()
            # self.v_debug_pub.publish(v_msg)

            # v_dot_msg = Float64MultiArray()
            # v_dot_msg.data = v_dot_.astype(float).flatten().tolist()
            # self.v_dot_debug_pub.publish(v_dot_msg)

            # p_msg = Float64MultiArray()
            # p_msg.data = p_.astype(float).flatten().tolist()
            # self.p_debug_pub.publish(p_msg)

            return

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """
        Callback function for the vehicle local position subscriber
        This function only store the fields of the message that are needed for the controller, and not the full message, this way when we run the controller we don't need to be creating arrays to send to the step of the model

        Parameters:
            msg (VehicleLocalPosition): The message received from the vehicle local position subscriber

        Returns:
            None
        """
        if not isinstance(msg, VehicleLocalPosition):
            raise TypeError("Message type is not VehicleLocalPosition")

        self.vehicle_pos = np.array(
            [msg.x, msg.y, msg.z]
        )  ## vehicle position in NED earth fixed frame in meters
        self.vehicle_velocity = np.array(
            [msg.vx, msg.vy, msg.vz]
        )  ## vehicle velocity in NED earth fixed frame in meters/second
        self.vehicle_acceleration = np.array(
            [msg.ax, msg.ay, msg.az]
        )  ## vehicle acceleration in NED earth fixed frame in meters/second^2
        self.get_logger().info(
            "Received message from vehicle local position {}".format(self.vehicle_pos)
        )
        # self.get_logger().info("Received message from vehicle local velocity {}".format(self.vehicle_velocity) )
        # self.get_logger().info("Received message from vehicle local acceleration {}".format(self.vehicle_acceleration) )

    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        """
        Callback function for the vehicle attitude subscriber
        This function only store the fields of the message that are needed for the controller, and not the full message, this way when we run the controller we don't need to be creating arrays to send to the step of the model

        Parameters:
            msg (VehicleAttitude): The message received from the vehicle attitude subscriber

        Returns:
            None
        """
        if not isinstance(msg, VehicleAttitude):
            raise TypeError("Message type is not VehicleAttitude")

        self.vehicle_attitude = np.array(
            [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]
        )  ## Vehicle attitude in quaternion form [x, y, z, w]

        self.vehicle_attitude = trf.Rotation.from_quat(self.vehicle_attitude)

    def vehicle_angular_velocity_callback(self, msg: VehicleAngularVelocity):
        """
        Callback function for the vehicle angular velocity subscriber
        This function only store the fields of the message that are needed for the controller, and not the full message, this way when we run the controller we don't need to be creating arrays to send to the step of the model

        Parameters:
            msg (VehicleAngularVelocity): The message received from the vehicle angular velocity subscriber

        Returns:
            None
        """
        if not isinstance(msg, VehicleAngularVelocity):
            raise TypeError("Message type is not VehicleAngularVelocity")

        self.vehicle_angular_speed = np.array(
            [msg.xyz[0], msg.xyz[1], msg.xyz[2]]
        )  ## Vehicle angular velocity in FRD body fixed frame in rad/s
        self.vehicle_angular_acceleration = np.array(
            [msg.xyz_derivative[0], msg.xyz_derivative[1], msg.xyz_derivative[2]]
        )  ## Vehicle angular acceleration in FRD body fixed frame in rad/s^2

    def sensors_combined_callback(self, msg: SensorCombined):
        """
        Callback function for the sensors combined subscriber

        This function saves the full data, as the controller will not needed, but might be useful for debugging purposes between the angular velocity and linear acceleration here and the attitude and angular velocity in the other callbacks

        Parameters:
            msg (SensorCombined): The message received from the sensors combined subscriber

        Returns:
            None
        """
        if not isinstance(msg, SensorCombined):
            raise TypeError("Message type is not SensorCombined")

        self.sensor_combined = msg


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
