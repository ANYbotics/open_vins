cmake_minimum_required(VERSION 3.5.1)

# Project name
project(ov_eval)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)


# check if we have our python libs files
# sudo apt-get install python-matplotlib python-numpy python2.7-dev
find_package(PythonLibs 2.7)
if(PYTHONLIBS_FOUND AND NOT DISABLE_MATPLOTLIB)
    add_definitions(-DHAVE_PYTHONLIBS=1)
    message(STATUS "PYTHON VERSION: " ${PYTHONLIBS_VERSION_STRING})
    message(STATUS "PYTHON INCLUDE: " ${PYTHON_INCLUDE_DIRS})
    message(STATUS "PYTHON LIBRARIES: " ${PYTHON_LIBRARIES})
endif()

# Describe catkin project
catkin_package(
  INCLUDE_DIRS
    src
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")

##################################################
# Make the core library
##################################################
add_library(${PROJECT_NAME}
  src/dummy.cpp
  src/alignment/AlignTrajectory.cpp
  src/alignment/AlignUtils.cpp
  src/calc/ResultTrajectory.cpp
  src/calc/ResultSimulation.cpp
  src/utils/Loader.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Specify your packages header files.
target_include_directories(${PROJECT_NAME} PRIVATE
  src
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

##################################################
# Make binary files!
##################################################

add_executable(pose_to_file
  src/pose_to_file.cpp
)

add_dependencies(pose_to_file
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pose_to_file
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(pose_to_file PRIVATE
  src
)

target_include_directories(pose_to_file SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



add_executable(format_converter
  src/format_converter.cpp
)

add_dependencies(format_converter
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(format_converter
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(format_converter PRIVATE
  src
)

target_include_directories(format_converter SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



add_executable(error_comparison
  src/error_comparison.cpp
)

add_dependencies(error_comparison
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(error_comparison
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(error_comparison PRIVATE
  src
)

target_include_directories(error_comparison SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



add_executable(error_dataset
  src/error_dataset.cpp
)

add_dependencies(error_dataset
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(error_dataset
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(error_dataset PRIVATE
  src
)

target_include_directories(error_dataset SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#================================================
add_executable(error_singlerun
  src/error_singlerun.cpp
)
add_dependencies(error_singlerun
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(error_singlerun
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(error_singlerun PRIVATE
  src
)

target_include_directories(error_singlerun SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#================================================
add_executable(error_simulation
  src/error_simulation.cpp
)

add_dependencies(error_simulation
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(error_simulation
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(error_simulation PRIVATE
  src
)

target_include_directories(error_simulation SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
#================================================
add_executable(timing_flamegraph
  src/timing_flamegraph.cpp
)

add_dependencies(timing_flamegraph
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(timing_flamegraph
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(timing_flamegraph PRIVATE
  src
)

target_include_directories(timing_flamegraph SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#================================================
add_executable(timing_comparison
  src/timing_comparison.cpp
)

add_dependencies(timing_comparison
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(timing_comparison
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(timing_comparison PRIVATE
  src
)

target_include_directories(timing_comparison SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#===========================================
add_executable(timing_percentages
  src/timing_percentages.cpp
)

add_dependencies(timing_percentages
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(timing_percentages
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(timing_percentages PRIVATE
  src
)

target_include_directories(timing_percentages SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#===========================================
add_executable(plot_trajectories
  src/plot_trajectories.cpp
)

add_dependencies(plot_trajectories
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(plot_trajectories
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(plot_trajectories PRIVATE
  src
)

target_include_directories(plot_trajectories SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

##################################################
# Python scripts!
##################################################
catkin_install_python(
  PROGRAMS
    python/pid_ros.py
    python/pid_sys.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
    pose_to_file
    format_converter
    error_comparison
    error_dataset
    error_singlerun
    error_simulation
    timing_flamegraph
    timing_comparison
    timing_percentages
    plot_trajectories
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
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



