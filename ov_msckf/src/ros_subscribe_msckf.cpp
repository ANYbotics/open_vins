/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "utils/VisualInertialOdometryRos.h"

#include <opencv2/core.hpp>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>


int main(int argc, char** argv) {
    // Launch our ros node
    ros::init(argc, argv, "visual_inertial_odometry");
    ros::NodeHandle nh("~");
    ros::NodeHandle image_nh;
    ros::CallbackQueue image_callback_queue;
    // Otherwise OpenCV will use  a lot of threads. https://medium.com/@rachittayal7/a-note-on-opencv-threads-performance-in-prod-d10180716fba.
    cv::setNumThreads(nh.param<int>("thread_count/opencv_thread_num", 2));
    ov_msckf::VisualInertialOdometryRos vio_ros_obj(nh, image_nh, image_callback_queue);

    ros::AsyncSpinner spinner(nh.param<int>("thread_count/global_thread_num", 1));
    spinner.start();

    ros::AsyncSpinner image_spinner(nh.param<int>("thread_count/image_specific_thread_num", 1), &image_callback_queue);
    image_spinner.start();

    std::thread callback_switch_thread;
    if (vio_ros_obj.enable_sensor_failure_handler_thread())
    {
        ROS_INFO_STREAM("Starting a callback_switch_thread for handling sensor failure.");
        callback_switch_thread = std::thread(&ov_msckf::VisualInertialOdometryRos::callback_switch, &vio_ros_obj);
    }

    ros::waitForShutdown();

    return EXIT_SUCCESS;
}



















