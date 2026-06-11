cmake_minimum_required(VERSION 3.10)
project(marvin_ros_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(marvin_msgs REQUIRED) 
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)

set(GENTO_SDK_ROOT "" CACHE PATH "Installed Gento SDK prefix, for example /usr/local")
set(GENTO_SDK_LIB "" CACHE FILEPATH "Path to installed libGentoSDK.so")
set(GENTO_SDK_INCLUDE_ROOT "" CACHE PATH "Path to installed Gento SDK headers, ending with include/gentosdk")

if(NOT GENTO_SDK_ROOT)
  if(DEFINED ENV{GENTO_SDK_ROOT} AND NOT "$ENV{GENTO_SDK_ROOT}" STREQUAL "")
    set(GENTO_SDK_ROOT "$ENV{GENTO_SDK_ROOT}")
  elseif(EXISTS "/usr/local/lib/libGentoSDK.so")
    set(GENTO_SDK_ROOT "/usr/local")
  endif()
endif()

if(NOT GENTO_SDK_LIB)
  if(EXISTS "${GENTO_SDK_ROOT}/lib/libGentoSDK.so")
    set(GENTO_SDK_LIB "${GENTO_SDK_ROOT}/lib/libGentoSDK.so")
  elseif(EXISTS "${GENTO_SDK_ROOT}/libGentoSDK.so")
    set(GENTO_SDK_LIB "${GENTO_SDK_ROOT}/libGentoSDK.so")
  endif()
endif()

if(NOT GENTO_SDK_INCLUDE_ROOT)
  if(EXISTS "${GENTO_SDK_ROOT}/include/gentosdk/L1Robot/L1Robot.h")
    set(GENTO_SDK_INCLUDE_ROOT "${GENTO_SDK_ROOT}/include/gentosdk")
  endif()
endif()

if(NOT EXISTS "${GENTO_SDK_LIB}")
  message(FATAL_ERROR
    "GENTO SDK dynamic library not found. Install the deb package first, or set "
    "GENTO_SDK_ROOT to the install prefix, for example /usr/local or /opt/gentosdk.")
endif()

if(NOT EXISTS "${GENTO_SDK_INCLUDE_ROOT}/L1Robot/L1Robot.h")
  message(FATAL_ERROR
    "GENTO SDK headers not found. Install the deb package first, or set "
    "GENTO_SDK_ROOT to the install prefix, for example /usr/local or /opt/gentosdk.")
endif()

get_filename_component(GENTO_SDK_LIB_DIR "${GENTO_SDK_LIB}" DIRECTORY)
message(STATUS "Using Gento SDK library: ${GENTO_SDK_LIB}")
message(STATUS "Using Gento SDK headers: ${GENTO_SDK_INCLUDE_ROOT}")

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  include
  ${GENTO_SDK_INCLUDE_ROOT}/Common
  ${GENTO_SDK_INCLUDE_ROOT}/L1Robot
  ${GENTO_SDK_INCLUDE_ROOT}/L0Control
  ${GENTO_SDK_INCLUDE_ROOT}/Kinematics
  ${GENTO_SDK_INCLUDE_ROOT}/Kinematics/ArmKinematics
  ${GENTO_SDK_INCLUDE_ROOT}/Kinematics/BaseMath
  ${GENTO_SDK_INCLUDE_ROOT}/Kinematics/DynaIdent
  ${GENTO_SDK_INCLUDE_ROOT}/Kinematics/KineCommon
  ${GENTO_SDK_INCLUDE_ROOT}/Kinematics/MotionPlanner
  ${GENTO_SDK_INCLUDE_ROOT}/Kinematics/SkyeBodyKinematics
)

add_executable(marvin_robot_node src/marvin_robot_node.cpp src/marvin_robot.cpp)
target_compile_definitions(marvin_robot_node PRIVATE CMPL_LIN)
target_link_libraries(marvin_robot_node "${GENTO_SDK_LIB}" pthread rt)
set_target_properties(marvin_robot_node PROPERTIES
  BUILD_RPATH "${GENTO_SDK_LIB_DIR}"
  INSTALL_RPATH "${GENTO_SDK_LIB_DIR}"
)
ament_target_dependencies(marvin_robot_node rclcpp sensor_msgs geometry_msgs std_msgs marvin_msgs tf2 std_srvs)


install(TARGETS
  marvin_robot_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/robot_jog_gui.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
