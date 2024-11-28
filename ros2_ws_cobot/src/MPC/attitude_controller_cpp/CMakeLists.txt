cmake_minimum_required(VERSION 3.8)
project(attitude_controller_cpp)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CASADI REQUIRED)

# include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIR}
  ${sensor_msgs_INCLUDE_DIRS}
)

set (executable_name
  controller
)

add_executable(${executable_name}
  src/controller.cpp
  src/ControllerMain.cpp
)


# Link against rclcpp and CasADi libraries
target_link_libraries(${executable_name}
  ${rclcpp_LIBRARIES}
  ${CASADI_LIBRARIES}  # Link CasADi libraries
  ${sensor_msgs_LIBRARIES}
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(CASADI)
ament_export_dependencies(sensor_msgs)

install (TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  system_parameters
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
