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
# SOFTWARE.

import casadi as ca
import spatial_casadi as sc
import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R


class RotationDynamicsNMPC:
    """
    Class for simulating the rotational dynamics of a rigid body
    """

    def __init__(
        self,
        J: np.ndarray,
        c: np.ndarray,
        A: np.ndarray,
        N: int = 10,
        T: float = 1 / 50,
    ):
        """
        Constructor for the NMPC class responsible for the control in attitude of the space cobot drone.

        Parameters:
            J : np.ndarray - Inertia matrix of the drone
            c : np.ndarray - Center of mass of the drone
            A : np.ndarray - Actuation matrix of the drone
            N : int - Size of the horizon for the NMPC
            T : float - The time step for the NMPC
        """
        self.J_ = J
        self.c_ = c
        self.A_ = A

        self.N = N
        self.T = T

        self.opts = {}

        self.problem_is_setup = False

    def setup(self):
        """
        Set up the optimization problem for the rotational dynamics.
        """
        if self.problem_is_setup:
            return

        self.opti = ca.Opti()

        self.w = self.opti.variable(3, self.N)  # angular velocity
        self.w_dot = self.opti.variable(3, self.N)  # angular acceleration
        self.q = self.opti.variable(4, self.N)  # quaternions
        self.u = self.opti.variable(6, self.N)  # actuation

        self.w0 = self.opti.parameter(3, 1)  # initial angular velocity
        self.w_dot0 = self.opti.parameter(3, 1)  # initial angular acceleration
        self.q0 = self.opti.parameter(
            4, 1
        )  # initial orientation in quaternion form 'xyzw'
        self.desired_rotation_matrix = self.opti.parameter(
            3, 3
        )  # desired rotation matrix

        # np.array only to check if the previous desired rotations matrix is the same as the current for the warm start of the optimization problem - THIS SHOULD NEVER BE USED IN THE SETUP FUNCTION ONLY IN THE STEP FUNCTION FOR VERIFICATION AND LATER UPDATE
        self.desired_r = np.identity(3)

        self.J = self.opti.parameter(3, 3)  # inertia matrix
        self.g = self.opti.parameter(3, 1)  # gravity
        self.A = self.opti.parameter(3, 6)  # actuation matrix
        self.skew_c = self.opti.parameter(3, 3)  # skew matrix of c

        # Inital parameters for initial conditions
        self.opti.set_value(self.w0, np.zeros(3))
        self.opti.set_value(self.w_dot0, np.zeros(3))
        self.opti.set_value(
            self.q0, R.from_euler("xyz", [0, 0, 0], degrees=True).as_quat()
        )
        self.opti.set_value(
            self.desired_rotation_matrix,
            R.from_euler("xyz", [0, 0, 0], degrees=True).as_matrix(),
        )

        # Values for the mechanical parameters of the system
        self.opti.set_value(self.J, self.J_)
        self.opti.set_value(
            self.g, np.array([0, 0, 0])
        )  ## When in simulation the gravity is zero
        ## Even tho we have values in the accelerometer we dont have changes in the body due to it, this is an abuse of notation, but otherwise the px4_sitl estimator will not work
        self.opti.set_value(self.A, self.A_)
        self.opti.set_value(self.skew_c, ca.skew(self.c_))

        # Initial conditions constraints
        self.opti._subject_to(self.w[:, 0] == self.w0)
        self.opti._subject_to(self.w_dot[:, 0] == self.w_dot0)
        self.opti._subject_to(self.q[:, 0] == self.q0)

        # Inital condition and boundaries for the optimization variables
        for i in range(0, self.N):
            self.opti.set_initial(
                self.q[:, i],
                sc.Rotation.from_euler("xyz", [0, 0, 0], degrees=True).as_quat(
                    seq="xyzw"
                ),
            )
            # Bound the thrust of each propeller to be between -2N and 2N
            for j in range(6):
                self.opti.subject_to(self.opti.bounded(-20, self.u[j, i], 20))

        # Dynamics of the system
        for i in range(0, self.N):
            c_r = sc.Rotation.from_quat(self.q[:, i], seq="xyzw").as_matrix()
            self.opti.subject_to(
                self.J @ self.w_dot[:, i]
                + ca.skew(self.w[:, i]) @ self.J @ self.w[:, i]
                - self.skew_c @ c_r.T @ self.g
                == self.A @ self.u[:, i]
            )

        # Integrate the quaternions and the angulas velocity based on the current angular velocity and acceleration
        for i in range(0, self.N - 1):
            self.opti.subject_to(
                self.q[:, i + 1]
                == self.quaternion_integration(self.q[:, i], self.w[:, i], self.T)
            )  ## Integration of the quaternion
            self.opti.subject_to(
                ca.norm_2(self.q[:, i + 1]) == 1
            )  ## Constraint to keep the quaternion normalized
            self.opti.subject_to(
                self.w[:, i + 1] == self.w[:, i] + self.T * self.w_dot[:, i]
            )  ## Integration of the angular velocity

        # Cost function
        @staticmethod
        def rotation_matrix_error(current_r, desired_r):
            """
            Computes the error between the current and desired rotation matrix

            Observation:
                This is an auxiliary function that can only be called from inside the setup function of the NMPC Class

            Parameters:
                current_r : sc.Rotation - current rotation matrix
                desired_r : sc.Rotation - desired rotation matrix

            Returns:
                ca.MX - error between the current and desired rotation
            """
            return ca.trace(ca.MX.eye(3) - (desired_r.T @ current_r))

        cost_function = 0
        for k in range(0, self.N):
            c_r = sc.Rotation.from_quat(self.q[:, k], seq="xyzw").as_matrix()
            cost_function += rotation_matrix_error(c_r, self.desired_rotation_matrix)
            cost_function += 0.1 * ca.sumsqr(self.u[:, k])

        self.opts = {
            "ipopt": {
                "print_level": 0,
                "tol": 1e-4,
                "acceptable_tol": 1e-4,
                "acceptable_iter": 10,
                "linear_solver": "mumps",
                "mu_strategy": "adaptive",
                "output_file": "solver.log",
            },
            "jit": True,
            "jit_cleanup": False,
            #'jit_options' : {'flags' : '-O1'},
            "print_time": 0,
        }

        self.opti.solver("ipopt", self.opts)

        self.opti.minimize(cost_function)

        self.problem_is_setup = True

    def set_ipopt_options(self):
        self.opts.update(
            {
                "ipopt": {
                    "print_level": 0,  # IPOPT verbosity (0 = silent)
                    "tol": 1e-4,  # Tolerance for convergence
                    "mu_strategy": "adaptive",  # Adaptive barrier parameter strategy
                    "linear_solver": "mumps",  # Linear solver choice (e.g., MUMPS)
                    "acceptable_tol": 1e-4,  # Tolerance for acceptable solution
                    "acceptable_iter": 10,  # Number of iterations for acceptable solution
                },
            }
        )
        return

    def set_jit(self):
        self.opts.update(
            {
                "jit": True,
                "jit_cleanup": False,
                "jit_options": {
                    "compiler": "ccache gcc"
                    #                'flags': ['-O2'],
                },
            }
        )

        return

    def step(self, w0: np.ndarray, w_dot0: np.ndarray, q0: np.ndarray, desired_r: R):
        """
        This function computes the optimal actuation for the current time step by solving an optimizations problem

        Parameters:
            w0 : np.ndarray : initial angular velocity
            w_dot0 : np.ndarray : initial angular acceleration

        Returns:
            np.ndarray : optimal actuation

        """

        if not self.problem_is_setup:
            raise Exception(
                "The problem has not been setup yet, call the setup function before calling the step function"
            )

        desired_rotation_matrix = sc.Rotation.from_euler(
            "xyz", desired_r.as_quat(), degrees=True
        ).as_matrix()
        desired_rotation_matrix = np.round(desired_rotation_matrix, decimals=2)

        self.opti.set_value(self.w0, w0)
        self.opti.set_value(self.w_dot0, w_dot0)
        self.opti.set_value(self.q0, q0)
        self.opti.set_value(self.desired_r, desired_rotation_matrix)

        if hasattr(self, "sol") and np.allclose(
            desired_rotation_matrix, self.desired_r
        ):
            q_norms = np.linalg.norm(self.sol.value(self.q), axis=0)
            quat = self.sol.value(self.q) / q_norms

            self.opti.set_initial(self.q, quat)
            self.opti.set_initial(self.w, self.sol.value(self.w))
            self.opti.set_initial(self.w_dot, self.sol.value(self.w_dot))
            self.opti.set_initial(self.u, self.sol.value(self.u))

        self.sol = self.opti.solve()
        self.desired_r = desired_rotation_matrix

        return self.sol

    @staticmethod
    def quaternion_integration(q, w, t):
        """
        Integrate quaternion q using angular velocity w over time t.

        Parameters:
            q: Quaternion.
            w: Angular velocity.
            t: Time step.

        Returns:
            Integrated quaternion.
        """
        epsilon = 1e-20  # to avoid division by zero
        w_ = w + epsilon
        w_norm = ca.norm_2(w_)

        p = ca.vertcat(
            w / (w_norm + epsilon) * ca.sin((w_norm + epsilon) * t / 2),
            ca.cos(w_norm * t / 2),
        )

        q_ = ca.vertcat(
            ca.horzcat(q[3], q[2], -q[1], q[0]),
            ca.horzcat(-q[2], q[3], q[0], q[1]),
            ca.horzcat(q[1], -q[0], q[3], q[2]),
            ca.horzcat(-q[0], -q[1], -q[2], q[3]),
        )

        return q_ @ p
