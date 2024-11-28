// Controller Node
#include "attitude_controller_cpp/controller.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <casadi/casadi.hpp>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Controller>()); 


}
