#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_navigation ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_navigation)

# Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)
add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-psabi)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: no build type specified, choosing Release")
  set(CMAKE_BUILD_TYPE Release)
endif()

#===============================================================================
#                              FIND DEPENDENCIES
#===============================================================================

# Find other Catkin packages required for build
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    avl_core
    avl_msgs
    avl_devices
)

# Find the Eigen3 package required for build
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Specify include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${manif_INCLUDE_DIRS}
)

#===============================================================================
#                              CONFIGURE PACKAGE
#===============================================================================

# Specify catkin-specific information to the build system
catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
    CATKIN_DEPENDS
        roscpp
        message_runtime
        std_msgs
        std_srvs
        geometry_msgs
        sensor_msgs
        avl_core
        avl_msgs
        avl_devices
    DEPENDS
    CFG_EXTRAS
)

# Create a variable for all exported targets for dependencies
set(EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#===============================================================================
#                                BUILD LIBRARIES
#===============================================================================

# Filter library
add_library(filt src/filter/ekf.cpp
                 src/filter/ukf.cpp
                 src/filter/ukf_add.cpp)
add_dependencies(filt ${EXPORTED_TARGETS})

# Navigation algorithm library
add_library(nav src/algorithm/inertial_nav.cpp
                src/algorithm/error_state.cpp
                src/algorithm/gm_process.cpp
                src/algorithm/sins_err_ekf.cpp
                src/algorithm/sins_ukf.cpp
                src/algorithm/sins_mukf_simple.cpp
                src/algorithm/sins_mukf_std.cpp
                src/algorithm/sins_mukf_dvl.cpp
                src/algorithm/sins_mukf_old.cpp
                src/algorithm/sins_mukf_gm.cpp
                src/algorithm/sins_mukf_scaled.cpp
                src/algorithm/gyrocompass_mukf.cpp
                src/algorithm/gyrocompass_ukf.cpp
                src/algorithm/gyrocompass_ukf2.cpp
                src/algorithm/moving_avg.cpp)

add_dependencies(nav ${EXPORTED_TARGETS})
target_link_libraries(nav filt ${catkin_LIBRARIES})

#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

# Nodes ------------------------------------------------------------------------

# Acoustic Ranging Node
add_executable(acoustic_ranging_node src/node/acoustic_ranging_node.cpp)
add_dependencies(acoustic_ranging_node ${EXPORTED_TARGETS})
target_link_libraries(acoustic_ranging_node node ${catkin_LIBRARIES})

# Depth Node
add_executable(depth_node src/node/depth_node.cpp)
add_dependencies(depth_node ${EXPORTED_TARGETS})
target_link_libraries(depth_node node ${catkin_LIBRARIES})

# Gyrocompass Node
add_executable(gyrocompass_node src/node/gyrocompass_node.cpp)
add_dependencies(gyrocompass_node ${EXPORTED_TARGETS})
target_link_libraries(gyrocompass_node node nav ${catkin_LIBRARIES})

# Inertial Nav Node
add_executable(inertial_nav_node src/node/inertial_nav_node.cpp)
add_dependencies(inertial_nav_node ${EXPORTED_TARGETS})
target_link_libraries(inertial_nav_node node nav comms ${catkin_LIBRARIES})

# Inertial Nav Postprocess Node
add_executable(inertial_nav_postprocess_node src/node/inertial_nav_postprocess_node.cpp)
add_dependencies(inertial_nav_postprocess_node ${EXPORTED_TARGETS})
target_link_libraries(inertial_nav_postprocess_node node nav ${catkin_LIBRARIES})

# Sensor Nav Node
add_executable(sensor_nav_node src/node/sensor_nav_node.cpp)
add_dependencies(sensor_nav_node ${EXPORTED_TARGETS})
target_link_libraries(sensor_nav_node node ${catkin_LIBRARIES})

# Maneuver Test Node
add_executable(maneuver_test_node src/node/maneuver_test_node.cpp)
add_dependencies(maneuver_test_node ${EXPORTED_TARGETS})
target_link_libraries(maneuver_test_node node nav ${catkin_LIBRARIES})

#===============================================================================
#                                  BUILD TESTS
#===============================================================================

#===============================================================================
#                                    INSTALL
#===============================================================================

# Mark executables and/or libraries for installation
install(TARGETS nav
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
