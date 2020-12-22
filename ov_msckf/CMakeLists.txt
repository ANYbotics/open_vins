cmake_minimum_required(VERSION 3.5.1)

# Project name
project(ov_msckf)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Create a list of catkin package dependencies used in header files of this package.
set(CATKIN_PACKAGE_HEADER_DEPENDENCIES
  cv_bridge
  geometry_msgs
  image_transport
  kindr_ros
  nav_msgs
  ov_core
  param_io
  rosbag
  roscpp
  sensor_msgs
  signal_logger_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  visualization_msgs
)

## Create a list of catkin package dependencies, now for both header and source files.
set(CATKIN_PACKAGE_DEPENDENCIES
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)

pkg_check_modules(TBB "tbb")
if (TBB_FOUND)
  add_definitions(
    ${TBB_CFLAGS}
    ${TBB_CFLAGS_OTHER}
  )
else()
  message([FATAL_ERROR] "tbb module not found")
endif ()

## The catkin_package macro generates cmake config files for your package.
catkin_package(
  INCLUDE_DIRS
    src
    ${EIGEN3_INCLUDE_DIR}
    ${TBB_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
  DEPENDS
    Boost
    kindr
    OpenCV
)


add_library(${PROJECT_NAME}
  src/sim/Simulator.cpp
  src/state/State.cpp
  src/state/StateHelper.cpp
  src/state/Propagator.cpp
  src/core/VioManager.cpp
  src/core/RosVisualizer.cpp
  src/update/UpdaterHelper.cpp
  src/update/UpdaterMSCKF.cpp
  src/update/UpdaterSLAM.cpp
  src/update/UpdaterZeroVelocity.cpp
  src/utils/helper_functions.cpp
  src/utils/VisualInertialOdometryRos.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Specify your packages header files.
target_include_directories(${PROJECT_NAME} PRIVATE
  src
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${TBB_LIBRARIES}
)

# Add executable
add_executable(run_serial_msckf
  src/ros_serial_msckf.cpp
)

add_dependencies(run_serial_msckf
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(run_serial_msckf PRIVATE
  src
)

target_include_directories(run_serial_msckf SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(run_serial_msckf
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Add executable
add_executable(run_subscribe_msckf
  src/ros_subscribe_msckf.cpp
)

add_dependencies(run_subscribe_msckf
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(run_subscribe_msckf PRIVATE
  src
)

target_include_directories(run_subscribe_msckf SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(run_subscribe_msckf
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


# Add executable
add_executable(run_simulation
  src/run_simulation.cpp
)

add_dependencies(run_simulation
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(run_simulation PRIVATE
  src
)

target_include_directories(run_simulation SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(run_simulation
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Add executable
add_executable(test_sim_meas
  src/test_sim_meas.cpp
)

add_dependencies(test_sim_meas
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(test_sim_meas PRIVATE
  src
)

target_include_directories(test_sim_meas SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(test_sim_meas
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Add executable
add_executable(test_sim_repeat
  src/test_sim_repeat.cpp
)

add_dependencies(test_sim_repeat
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(test_sim_repeat PRIVATE
  src
)

target_include_directories(test_sim_repeat SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(test_sim_repeat
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Installation
install(
  TARGETS
    ${PROJECT_NAME}
    run_serial_msckf
    run_subscribe_msckf
    run_simulation
    test_sim_meas
    test_sim_repeat
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/empty_test.cpp
  )

  target_include_directories(test_${PROJECT_NAME}
    PRIVATE
      src
    SYSTEM PUBLIC
      ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME}
    )
  endif(cmake_code_coverage_FOUND)
endif()


