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


import sympy as sp
import numpy as np


class ThrustPwmConvert:
    """
    This simple class will be used to have an easy interface in converting between thurst and pwm, since we only have acess to one model that converts
    for pwm (us) -> thrust (N), and we will need to make the oposite conversion from thrust -> pwm
    """

    def __init__(
        self,
        pwm_to_thrust_coefs: np.array,
        pwm_limits: list,
        engine_off_pwm: float = 1500.0,
    ):
        """
        Constructor for the class

        Initializas the default values for the function, and stores the general solution we will need to use in the reverse conversion of the mode

        Parameters:
            pwm_to_thrust_coefs - np.array : This is as ordered list with the coeficients of the polynomial that converts from pwm -> thrust.
                                The coeficient should be ordered in crescent order, so [a b c], where thrust = a + b * pwm + c * pwm^(2).
                                If a polynomial with an higher order is used, the same order should be applied, and the coeficients should be append at the end

            pwm_limits - list <float> : A list that tell us the range of values we are allowed to give as pwm, to make sure we are operating inse the correct values for the current model
                                        and to make sure we dont give a value that is out of bounds for waht the motor allows us

            engine_off_pwm - float :  The pwm we must give to turn the engine off, meaning no rotation, this will be given when we can product a value that is inside bounds for the desired thrust

        Returns
            obj - ThrustPwmConvert : The new object of type ThrustToPwmConvert created
        """

        self.pwm_limits = pwm_limits
        if len(self.pwm_limits) != 2:
            exit(1)

        self.engine_off_pwm = engine_off_pwm

        ## Save and create and numpy array with the coeficient of the used
        self.pwm_to_thrust_coefs = pwm_to_thrust_coefs

        ## Create the sympy symbols that are needed for the equation
        self.pwm, self.thrust = sp.symbols("x y")

        ## Create a vector that is [pwm⁰ pwm¹ pwm²] to then multiply with the the coeficent and create the equation in a faster way
        self.order = np.size(self.pwm_to_thrust_coefs)
        pwm_vec = np.array([self.pwm ** (i) for i in range(0, self.order)])

        self.eq = (pwm_vec @ self.pwm_to_thrust_coefs.T) - self.thrust

        self.general_solution = sp.solve(self.eq, self.pwm)

    def thrust_to_pwm(self, thrust: float):
        if np.abs(thrust) < 0.1:
            return self.engine_off_pwm

        specific_solution = [
            sol.subs(self.thrust, thrust) for sol in self.general_solution
        ]

        min_pwm_allowed = self.pwm_limits[0]
        max_pwm_allowed = self.pwm_limits[1]
        desired_pwm = [
            pwm
            for pwm in specific_solution
            if min_pwm_allowed <= pwm and pwm <= max_pwm_allowed
        ]

        ## This condition checks that we have at least one pwm value that is allowed, and if so, we return the command that turns off the engine
        if len(desired_pwm) < 1:
            return self.engine_off_pwm

        ## Return the first value of pwm that will produce a certain thrust, if more than one exist (should not be possible), we will return only on of them
        return desired_pwm[0]

    def pwm_to_thrust(self, pwm: float):
        """
        Get the correspondent thrust value allowed for a desired pwm value

        Parameters:
            pwm - float : Desired pwm value to be applied in the motor

        Return
            Thrust - float : The thrust value that a motor will be produccing according to the current
        """
        min_pwm_allowed = self.pwm_limits[0]
        max_pwm_allowed = self.pwm_limits[1]

        if min_pwm_allowed >= pwm or pwm >= max_pwm_allowed:
            return self.engine_off_pwm

        pwm_vec = np.array([pwm ** (i)] for i in range(0, self.order))

        return pwm_vec.T * self.pwm_to_thrust_coefs
