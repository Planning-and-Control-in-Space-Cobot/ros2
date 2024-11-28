/*
MIT License

Copyright (c) 2024 [Your Name]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef __ROTATION_CASADI__
#define __ROTATION_CASADI__

#include <vector>
#include <string>
#include <stdexcept>

#include <casadi/casadi.hpp>

/**
 * @class RotationCasadi
 * @brief A class that represents a rotation using a quaternion and provides
 *        methods to convert the quaternion to Euler angles and a rotation matrix.
 */
class RotationCasadi {
public:
    /**
     * @brief Default constructor for the RotationCasadi class.
     */
    RotationCasadi() = default;

    /**
     * @brief Default destructor for the RotationCasadi class.
     */
    ~RotationCasadi() = default;

    /**
     * @brief Sets the quaternion of the RotationCasadi object.
     *
     * @param q A 4x1 CasaDI MX representation of a quaternion
     * @param seq A std::string representing the sequence of the quaternion elements, the default is "xyzw".
     *            The available options are "xyzw" and "wxyz". 
     * 
     * @throw std::invalid_argument if the quaternion is not a 4x1 matrix.
     * @throw std::invalid_argument if the sequence is not "xyzw" or "wxyz".
     */
    inline void fromQuat(casadi::MX q, std::string seq = "xyzw") {
        if (q.size1() != 4 || q.size2() != 1) {
            throw std::invalid_argument("Quaternion must be a 4x1 matrix.");
        }

        if (seq == "xyzw") {
            this->qx = q(0);
            this->qy = q(1);
            this->qz = q(2);
            this->qw = q(3);
        } else if (seq == "wxyz") {
            this->qx = q(1);
            this->qy = q(2);
            this->qz = q(3);
            this->qw = q(0);
        } else {
            throw std::invalid_argument("Invalid sequence. The available options are 'xyzw' and 'wxyz'.");
        }
        return;
    }

    /**
     * @brief Gets the quaternion of the RotationCasadi object.
     *
     * @param seq A std::string representing the sequence of the quaternion elements, the default is "xyzw".
     *           The available options are "xyzw" and "wxyz".
     *
     * @return A 4x1 CasADi MX representing the quaternion.
     * 
     * @throw std::runtime_error if the quaternion is not set.
     * @throw std::invalid_argument if the sequence is not "xyzw" or "wxyz".
     */
    inline casadi::MX toQuat(std::string seq = "xyzw") {
        if (seq == "xyzw") {
            return casadi::MX::vertcat(
                {
                    this->qx, 
                    this->qy,
                    this->qz, 
                    this->qw
                });
        } else if (seq == "wxyz") {
            return casadi::MX::vertcat(
                {
                    this->qw,
                    this->qx,
                    this->qy,
                    this->qz
                });
        } else {
            throw std::invalid_argument("Invalid sequence. The available options are 'xyzw' and 'wxyz'.");
        }
    }

    /**
     * @brief Converts the quaternion to a rotation matrix.
     * 
     * @return A 3x3 CasADi MX representing the rotation matrix.
     * 
     * @throw std::runtime_error if the quaternion is not set.
     */
    inline casadi::MX toRotationMatrix() {
        auto x = this->qx;
        auto y = this->qy;
        auto z = this->qz;
        auto w = this->qw;

        auto x2 = x * x;
        auto y2 = y * y;
        auto z2 = z * z;
        auto w2 = w * w;

        auto xy = x * y;
        auto zw = z * w;
        auto xz = x * z;
        auto yw = y * w;
        auto yz = y * z;
        auto xw = x * w;

        auto R = casadi::MX::horzcat({
            casadi::MX::vertcat( {
                x2 - y2 - z2 + w2,
                2.0 * (xy + zw),
                2.0 * (xz - yw)
            }), 
            casadi::MX::vertcat({
                2.0 * (xy - zw),
                -x2 + y2 - z2 + w2,
                2.0 * (yz + xw)
            }),
            casadi::MX::vertcat({
                2.0 * (xz + yw),
                2.0 * (yz - xw),
                -x2 - y2 + z2 + w2
            })
        });

        return R;
    }

private:
    // Quaternion [x, y, z, w]
    casadi::MX qx;
    casadi::MX qy; 
    casadi::MX qz;
    casadi::MX qw;
};

#endif // __ROTATION_CASADI__