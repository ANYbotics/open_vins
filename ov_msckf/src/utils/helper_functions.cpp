/*!
 * @file    helper_functions.cpp
 * @author  Guoxiang Zhou (ANYbotics)
 * @date    17.12.20
 * @brief   The implementation of helper functions for VIO
 */

#include "helper_functions.h"

namespace ov_msckf {

geometry_msgs::PoseStamped tfToPose(const tf::StampedTransform& tf)
{
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = tf.frame_id_;
    poseStamped.header.stamp = tf.stamp_;
    poseStamped.pose.position.x = tf.getOrigin().getX();
    poseStamped.pose.position.y = tf.getOrigin().getY();
    poseStamped.pose.position.z = tf.getOrigin().getZ();
    poseStamped.pose.orientation.w = tf.getRotation().getW();
    poseStamped.pose.orientation.x = tf.getRotation().getX();
    poseStamped.pose.orientation.y = tf.getRotation().getY();
    poseStamped.pose.orientation.z = tf.getRotation().getZ();
    return poseStamped;
}

} // namespace ov_msckf
