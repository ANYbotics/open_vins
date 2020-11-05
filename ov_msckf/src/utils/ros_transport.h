#pragma once

// ros
#include <ros/ros.h>

// param io
#include <param_io/get_param.hpp>

namespace ov_msckf {

/**
 * Advertise a ROS topic.
 *
 * @tparam MsgT           Message type.
 * @param nodeHandle      ROS node handle.
 * @param publisher       ROS publisher/keyword.
 * @param key             Publisher name/key.
 */
template<typename MsgT>
void advertise(ros::NodeHandle &nodeHandle, ros::Publisher &publisher, const std::string &key) {
    publisher = nodeHandle.advertise<MsgT>(param_io::param<std::string>(nodeHandle, "publishers/" + key + "/topic", key),
                                           param_io::param<uint32_t>(nodeHandle, "publishers/" + key + "/queue_size", 1),
                                           param_io::param<bool>(nodeHandle, "publishers/" + key + "/latch", false));
}
}

