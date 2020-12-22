/*!
 * @file    helper_functions.hpp
 * @author  Guoxiang Zhou (ANYbotics)
 * @date    17.12.20
 * @brief   The interface of helper functions for VIO
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

namespace ov_msckf {

/**
 * @brief Convert tf::StampedTransform to geometry_msgs::PoseStamped.
 * @param tf  tf::StampedTransform message.
 * @return geometry_msgs::PoseStamped message.
 */
geometry_msgs::PoseStamped tfToPose(const tf::StampedTransform& tf);

} // namespace ov_msckf