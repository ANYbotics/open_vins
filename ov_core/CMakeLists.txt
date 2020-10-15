cmake_minimum_required(VERSION 3.5.1)

# Project name
project(ov_core)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Create a list of catkin package dependencies used in header files of this package.
set(CATKIN_PACKAGE_HEADER_DEPENDENCIES
  cv_bridge
  geometry_msgs
  nav_msgs
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  tf
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

## Find system dependencies providing a CMake module.
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

## The catkin_package macro generates cmake config files for your package.
catkin_package(
  INCLUDE_DIRS
    src
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
  DEPENDS
    Boost
    OpenCV
)

add_library(${PROJECT_NAME}
  src/ov_core/dummy.cpp
  src/ov_core/init/InertialInitializer.cpp
  src/ov_core/sim/BsplineSE3.cpp
  src/ov_core/track/TrackBase.cpp
  src/ov_core/track/TrackAruco.cpp
  src/ov_core/track/TrackDescriptor.cpp
  src/ov_core/track/TrackKLT.cpp
  src/ov_core/track/TrackSIM.cpp
  src/ov_core/types/Landmark.cpp
  src/ov_core/feat/Feature.cpp
  src/ov_core/feat/FeatureInitializer.cpp
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
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
    # Testing targets
  add_executable(test_tracking
    src/ov_core/test_tracking.cpp
  )
  add_dependencies(test_tracking
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_tracking
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
  target_include_directories(test_tracking PRIVATE
    src
  )
  target_include_directories(test_tracking SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIRS}
  )


  add_executable(test_webcam
    src/ov_core/test_webcam.cpp)

  add_dependencies(test_webcam
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(test_webcam
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )

  target_include_directories(test_webcam PRIVATE
    src
  )

  target_include_directories(test_webcam SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIRS}
  )



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
  endif()
endif()



