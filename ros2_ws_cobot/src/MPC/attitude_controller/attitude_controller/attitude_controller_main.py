# MIT License
#
# Copyright (c) 2024 Andre Rebelo Teixeira
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.import rclpy

## Ros2 modules
import rclpy
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
from px4_msgs.msg import SensorCombined, VehicleAngularVelocity, VehicleAttitude

# STD messages
from std_msgs.msg import Float64MultiArray

# Actuators messages
from actuator_msgs.msg import Actuators

## Self defined modules
from attitude_controller.controller import RotationDynamicsNMPC as NMPC
from attitude_controller.ThrustPwmConvert import ThrustPwmConvert as ThrustPwmConvert
from attitude_controller.ThrustToRpm import ThrustToRpm, MotorType

## Python modules
import os
import time
import numpy as np
import sympy as sp
from scipy.spatial.transform import Rotation as R


class AttitudeController(Node):
    def __init__(self):
        """
        Initialize the attitude controller node

        Parameters
            None

        Returns
            None
        """
        super().__init__("predictive_attitude_controller")

        ## Declare the subscriber
        self.sensors_combined_subscriber = self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self.sensors_combined_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.vehicle_angular_velocity_subscriber = self.create_subscription(
            VehicleAngularVelocity,
            "/fmu/out/vehicle_angular_velocity",
            self.vehicle_angular_velocity_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.vehicle_attitude_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        ## Declare the publisher
        self.actuation_publisher = self.create_publisher(
            Actuators, "/space_cobot0/motor/actuator_cmd", 10
        )

        ## Declare the parameters
        self.declare_parameter("controller_dt", 0.1)  # 10 Hz = 0.1 s
        self.declare_parameter("desired_attitude", [0.0, 0.0, 0.0])
        self.declare_parameter("J_matrix_path", "no_file")
        self.declare_parameter("c_vector_path", "no_file")
        self.declare_parameter("A_matrix_path", "no_file")
        self.declare_parameter("horizon", 20)
        self.declare_parameter("time_step", 0.1)

        ## Declare the thrust model parameters
        self.declare_parameter("thurst_model_path", "no_file")

        ## Get the parameter value if it was changed at launch
        self.controller_dt = (
            self.get_parameter("controller_dt").get_parameter_value().double_value
        )
        self.desired_attitude = (
            self.get_parameter("desired_attitude")
            .get_parameter_value()
            .double_array_value
        )
        self.J_matrix_path = (
            self.get_parameter("J_matrix_path").get_parameter_value().string_value
        )
        self.c_vector_path = (
            self.get_parameter("c_vector_path").get_parameter_value().string_value
        )
        self.A_matrix_path = (
            self.get_parameter("A_matrix_path").get_parameter_value().string_value
        )
        self.horizon = self.get_parameter("horizon").get_parameter_value().integer_value
        self.time_step = (
            self.get_parameter("time_step").get_parameter_value().double_value
        )
        self.thurst_model_path = (
            self.get_parameter("thurst_model_path").get_parameter_value().string_value
        )

        if (
            self.J_matrix_path == "no_file"
            or os.path.isfile(self.J_matrix_path) == False
        ):
            self.get_logger().warn(
                "The J matrix path is either not valid or not set. Using the default file provided with the package"
            )
            package_name = "attitude_controller"
            package_share_directory = get_package_share_directory(package_name)
            self.J_matrix_path = os.path.join(
                package_share_directory, "configs", "J_matrix.npy"
            )

        if (
            self.c_vector_path == "no_file"
            or os.path.isfile(self.c_vector_path) == False
        ):
            self.get_logger().warn(
                "The c vector path is either not valid or not set. Using the default file provided with the package"
            )
            package_name = "attitude_controller"
            package_share_directory = get_package_share_directory(package_name)
            self.c_vector_path = os.path.join(
                package_share_directory, "configs", "c_vector.npy"
            )

        if (
            self.A_matrix_path == "no_file"
            or os.path.isfile(self.A_matrix_path) == False
        ):
            self.get_logger().warn(
                "The A matrix path is either not valid or not set. Using the default file provided with the package"
            )
            package_name = "attitude_controller"
            package_share_directory = get_package_share_directory(package_name)
            self.A_matrix_path = os.path.join(
                package_share_directory, "configs", "A_matrix.npy"
            )

        if (
            self.thurst_model_path == "no_file"
            or os.path.isfile(self.thurst_model_path) == False
        ):
            self.get_logger().warn(
                "No file path, or invalid file path for thrust model polynomial. Using the default file provided with the package"
            )
            package_name = "attitude_controller"
            package_share_directory = get_package_share_directory(package_name)
            self.thurst_model_path = os.path.join(
                package_share_directory, "configs", "thrust_model.npy"
            )

            self.get_logger().warn(
                "No file path, or invalid file path for CW thrust model polynomial. Using the default file provided with the package"
            )
            package_name = "attitude_controller"
            package_share_directory = get_package_share_directory(package_name)
            self.cw_thurst_model_path = os.path.join(
                package_share_directory, "configs", "cw_thrust_model.npy"
            )

        ## Load the matrices
        J = np.load(self.J_matrix_path)
        c = np.load(self.c_vector_path)
        A = np.load(self.A_matrix_path)

        thrust_model_ = np.load(self.thurst_model_path)

        max_thrust = 2  # Newtons
        min_thrust = -2  # Newtons

        ## Motor configuration, this will be important to know how to convert from thrust to RPM
        self.motor_configuration = [
            MotorType.CCW,
            MotorType.CW,
            MotorType.CCW,
            MotorType.CW,
            MotorType.CCW,
            MotorType.CW,
        ]

        self.Thrust_to_rpm_converter = ThrustToRpm(
            thrust_model_, min_thrust, max_thrust
        )

        ## Remove the first 3 rows of A, since this is relative to the position dynamics rather than the orientation dynamics
        A = A[3:, :]

        self.controller = NMPC(J, c, A, self.horizon, self.time_step)
        self.controller.setup()
        self.timer = self.create_timer(self.controller_dt, self.controller_step)
        self.desired_attitude = np.array([20, 20, 20])
        self.w = None
        self.w_dot = None
        self.q = None

        ## System state variables

    def sensors_combined_callback(self, msg: SensorCombined) -> None:
        """
        Callback for the sensor combined topic, more information on this message here
        https://docs.px4.io/main/en/msg_docs/SensorCombined.html

        Parameters
            msg - px4_msgs.msg.SensorCombined: the sensor combined message

        Returns
            None
        """
        if type(msg) != SensorCombined:
            self.get_logger().error(
                "The message received is not of the correct type. Expected SensorCombined, received {}".format(
                    type(msg)
                )
            )
            return

        self.v = np.array(
            [
                msg.accelerometer_m_s2[0],
                msg.accelerometer_m_s2[1],
                msg.accelerometer_m_s2[2],
            ]
        )
        self.get_logger().warn("Acceleration: {}".format(self.v))
        return

    def vehicle_angular_velocity_callback(self, msg: VehicleAngularVelocity) -> None:
        """
        Callback for the vehicle angular velocity topic, more information on this message here
        https://docs.px4.io/main/en/msg_docs/VehicleAngularVelocity.html

        Parameters
            msg - px4_msgs.msg.VehicleAngularVelocity: the vehicle angular velocity message

        Returns
            None
        """
        if type(msg) != VehicleAngularVelocity:
            self.get_logger().error(
                "The message received is not of the correct type. Expected VehicleAngularVelocity, received {}".format(
                    type(msg)
                )
            )
            return

        self.w = np.array([msg.xyz[0], msg.xyz[1], msg.xyz[2]])
        self.w_dot = np.array(
            [msg.xyz_derivative[0], msg.xyz_derivative[1], msg.xyz_derivative[2]]
        )
        self.get_logger().warn("Angular velocity: {}".format(self.w))
        self.get_logger().warn("Angular acceleration: {}".format(self.w_dot))
        return

    def vehicle_attitude_callback(self, msg: VehicleAttitude) -> None:
        """
        Callback for the vehicle attitude topic, more information on this message here
        https://docs.px4.io/main/en/msg_docs/VehicleAttitude.html

        Parameters
            msg - px4_msgs.msg.VehicleAttitude: the vehicle attitude message

        Returns
            None
        """
        if type(msg) != VehicleAttitude:
            self.get_logger().error(
                "The message received is not of the correct type. Expected VehicleAttitude, received {}".format(
                    type(msg)
                )
            )
            return

        self.q = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
        self.get_logger().warn("Quaternion: {}".format(self.q))

        return

    def desired_attitude_callback(self, msg: Float64MultiArray) -> None:
        """
        Callback or the desired attitude topic, to make sure we can
        change the values of the attitude controller in real time

        Parameters
            msg - std_msgs.msg.Float64MultiArray: the desired attitude

        Returns
            None
        """

        if len(msg.data) == 3:
            self.desired_attitude = np.array(msg.data)
        else:
            self.get_logger().warn(
                "The desired attitude must be a 3D vector. Value not updated."
            )
            self.get_logger().warn("Current value: {}".format(self.desired_attitude))
        return

    def controller_step(self):
        """
        Compute the next actuation value to send to the motors

        Parameters
            None

        Returns
            actuation - list: the actuation value to send to the motors
        """
        ## get actuation in rps (rotation per second)
        if self.w is not None and self.w_dot is not None and self.q is not None:
            start = time.time_ns()
            desired_rotation = R.from_euler("xyz", self.desired_attitude, degrees=True)
            sol = self.controller.step(self.w, self.w_dot, self.q, desired_rotation)
            end = time.time_ns()

            self.get_logger().warn(
                "Time (ms) to solve the optimization problem: {}".format(
                    (end - start) * 1e-6
                )
            )

            actuation = sol.value(self.controller.u[:, 1])

            msg = Actuators()
            msg.header.stamp = self.get_clock().now().to_msg()
            ## convert thrust to pwm

            for i, thrust in enumerate(actuation):
                rpm = self.Thrust_to_rpm_converter.get_rpm(thrust)
                msg.velocity.append(rpm if rpm is not None else 0)

            self.get_logger().warn("Actuation: {}".format(actuation))
            self.get_logger().warn("Actuation in RPM: {}".format(msg.velocity))

            u_computed = sol.value(self.controller.u[:, :])
            self.get_logger().warn("Computed control input: {}".format(u_computed))
            q_computed = sol.value(self.controller.q[:, :])
            self.get_logger().warn("Computed quaternion: {}".format(q_computed))

            ## publish the pwm instead of returning it
            self.actuation_publisher.publish(msg)

        else:
            self.get_logger().warn(
                "Some data is still unavailable. Cannot compute the actuation value."
            )
            return np.zeros((6, 1))


def main(args=None):
    rclpy.init(args=args)
    attitude_controller = AttitudeController()
    rclpy.spin(attitude_controller)
    attitude_controller.destroy_node()
    rclpy.shutdown()
