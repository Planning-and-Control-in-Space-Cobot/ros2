#ifndef __CONTROLLER_PIPELINE__HPP__
#define __CONTROLLER_PIPELINE__HPP__

#include <cstdlib>
#include <cmath>

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ControllerPipeline {
    friend class Controller;
public:
    ControllerPipeline() = default;
    ~ControllerPipeline() = default;


    /**
        * @brief Update the IMU message
        *
        * @param msg The IMU message
        *
        * @return void
        *
     */
    inline void updateImuMsg(const std::shared_ptr<const sensor_msgs::msg::Imu>   msg) {
        imu_msg = std::const_pointer_cast<sensor_msgs::msg::Imu>(msg);
    }


    /**
        * @brief Get the IMU linear acceleration
        *
        * @return The IMU linear acceleration
        *
     */
    inline std::vector<double> getImuAngularVelocity() {
        if (imu_msg == nullptr) {
            return {0, 0, 0};
        }
        return {imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z};
    }

    /**
        * @brief Get the IMU linear acceleration
        *
        * @return The IMU linear acceleration
        *
     */
    inline std::vector <double> getImuLinearAcceleration() {
        if  (imu_msg == nullptr) {
            return {0, 0, 0};
        }
        return {imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z};
    }

    /**
        * @brief Get the IMU orientation
        *
        * @return The IMU orientation
        *
     */
    inline std::vector <double> getImuOrientation() {
        if (imu_msg == nullptr) {
            return {0, 0, 0, 1};
        }
        return {imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w};
    }

    /**
        * @brief Get the IMU header
        *
        * @return The IMU header
        *
     */
    inline std_msgs::msg::Header getImuHeader() {
        if (imu_msg == nullptr) {
            return std_msgs::msg::Header();
        }
        return imu_msg->header;
    }

    /**
        * @brief Get the IMU message
        *
        * @return The IMU message
        *
     */
    inline sensor_msgs::msg::Imu::SharedPtr getImuMsg() {
        return imu_msg;
    }

    /**
        * @brief Check if the IMU message is valid
        *
        * @return True if the IMU message is valid, false otherwise
        *
     */
    inline bool hasImuMsg() {
        return imu_msg != nullptr;
    }

    /**
        * @brief Update the desired orientation if the receive message is a valid quaternion
        *
        * @param msg The desired orientation
        *
        * @return void
        *
     */
    inline void updateDesiredOrientation(const std::shared_ptr<const std_msgs::msg::Float64MultiArray> msg) {
        std::vector <double> quat = msg->data;

        // Verify that the quaternion has 4 elements
        if (quat.size() != 4) {
            return;
        }

        double x = quat[0];
        double y = quat[1];
        double z = quat[2]; 
        double w = quat[3];

        double norm = std::sqrt(x * x + y * y + z * z + w * w);

        double max_tolerance = 1e-3;

        // This check that the norm of the quaternion is either close to 1, or close to 0
        if (std::abs(norm - 1) >  max_tolerance) {
            return;
        }

        //  This makes sure that the norm cannot be close to zero, and there for, we only update if our norm is close to 1
        if (norm < 1e-3) {
            return;
        }

        // Update the quaternion
        desired_orientation = quat;
    }

    /**
        * @brief Get the desired orientation 
        *
        * @return The desired orientation
        *
     */
    inline std::vector<double> getDesiredOrientation() {
        return desired_orientation;
    }

private:
    sensor_msgs::msg::Imu::SharedPtr imu_msg = nullptr; 
    std::vector <double> desired_orientation = {0, 0, 0, 1}; // Identity quaternion is the default value for the desired orientation
};

#endif // __CONTROLLER_PIPELINE__HPP__