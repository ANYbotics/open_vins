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


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/date_time/microsec_time_clock.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "core/RosVisualizer.h"
#include <ov_core/utils/dataset_reader.h>
#include <signal_logger_msgs/UInt32Stamped.h>
#include "utils/parse_ros.h"

// param io
#include <param_io/get_param.hpp>


using namespace ov_msckf;


VioManager* sys;
RosVisualizer* viz;


// Buffer data
double time_buffer = -1;
cv::Mat img0_buffer, img1_buffer;

// Callback functions
void callback_inertial(const sensor_msgs::Imu::ConstPtr& msg);
void callback_monocular(const sensor_msgs::ImageConstPtr& msg0);
void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1);
void callback_feet_contact(const signal_logger_msgs::UInt32Stamped::ConstPtr& msg0, const signal_logger_msgs::UInt32Stamped::ConstPtr& msg1, const signal_logger_msgs::UInt32Stamped::ConstPtr& msg2, const signal_logger_msgs::UInt32Stamped::ConstPtr& msg3);



// Main function
int main(int argc, char** argv) {

    // Launch our ros node
    ros::init(argc, argv, "visual_inertial_odometry");
    ros::NodeHandle nh("~");
    ros::NodeHandle imageNh;
    // Use a separate callback queue for images input so that the system can achieve real-time performance.
    ros::CallbackQueue imageCallbackQueue;
    imageNh.setCallbackQueue(&imageCallbackQueue);


    // Create our VIO system. Read parameters from ROS parameter server.
    VioManagerOptions params = parse_ros_nodehandler(nh);
    sys = new VioManager(params);
    viz = new RosVisualizer(nh, sys);


    //===================================================================================
    //===================================================================================
    //===================================================================================
    std::string imu_sensor = "imu_sensor";
    std::string camera_input_1 = "wide_angle_camera_front";
    std::string camera_input_2 = "wide_angle_camera_rear";
    std::string contact_force_lf_foot = "contact_force_lf_foot";
    std::string contact_force_rf_foot = "contact_force_rf_foot";
    std::string contact_force_lh_foot = "contact_force_lh_foot";
    std::string contact_force_rh_foot = "contact_force_rh_foot";

    // Logic for sync stereo subscriber
    // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
    message_filters::Subscriber<sensor_msgs::Image> image_sub1(imageNh, param_io::param<std::string>(nh, "subscribers/" + camera_input_1 + "/topic", camera_input_1),
                                                               param_io::param<uint32_t>(nh, "subscribers/" + camera_input_1 + "/queue_size", 1));
    message_filters::Subscriber<sensor_msgs::Image> image_sub2(imageNh,param_io::param<std::string>(nh, "subscribers/" + camera_input_2 + "/topic", camera_input_2),
                                                               param_io::param<uint32_t>(nh, "subscribers/" + camera_input_2 + "/queue_size", 1));
    //message_filters::TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image> sync(image_sub0,image_sub1,5);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(5), image_sub1, image_sub2);

    // Create subscribers
    ros::Subscriber subimu = nh.subscribe(param_io::param<std::string>(nh, "subscribers/" + imu_sensor + "/topic", imu_sensor),
                                          param_io::param<uint32_t>(nh, "subscribers/" + imu_sensor + "/queue_size", 1), callback_inertial);

    // Create feet contact subscriber.
    message_filters::Subscriber<signal_logger_msgs::UInt32Stamped> foot_lf_sub(nh,param_io::param<std::string>(nh, "subscribers/" + contact_force_lf_foot + "/topic", contact_force_lf_foot),
                                                                               param_io::param<uint32_t>(nh, "subscribers/" + contact_force_lf_foot + "/queue_size", 1));
    message_filters::Subscriber<signal_logger_msgs::UInt32Stamped> foot_lh_sub(nh,param_io::param<std::string>(nh, "subscribers/" + contact_force_lh_foot + "/topic", contact_force_lh_foot),
                                                                               param_io::param<uint32_t>(nh, "subscribers/" + contact_force_lh_foot + "/queue_size", 1));
    message_filters::Subscriber<signal_logger_msgs::UInt32Stamped> foot_rf_sub(nh,param_io::param<std::string>(nh, "subscribers/" + contact_force_rf_foot + "/topic", contact_force_rf_foot),
                                                                               param_io::param<uint32_t>(nh, "subscribers/" + contact_force_rf_foot + "/queue_size", 1));
    message_filters::Subscriber<signal_logger_msgs::UInt32Stamped> foot_rh_sub(nh,param_io::param<std::string>(nh, "subscribers/" + contact_force_rh_foot + "/topic", contact_force_rh_foot),
                                                                               param_io::param<uint32_t>(nh, "subscribers/" + contact_force_rh_foot + "/queue_size", 1));
    typedef message_filters::sync_policies::ExactTime<signal_logger_msgs::UInt32Stamped, signal_logger_msgs::UInt32Stamped, signal_logger_msgs::UInt32Stamped, signal_logger_msgs::UInt32Stamped> feet_contact_sync_pol;
    message_filters::Synchronizer<feet_contact_sync_pol> feet_contact_sync(feet_contact_sync_pol(9999), foot_lf_sub,foot_lh_sub, foot_rf_sub, foot_rh_sub);
    feet_contact_sync.registerCallback(boost::bind(&callback_feet_contact, _1, _2, _3, _4));


    ros::Subscriber subcam;
    if(params.state_options.num_cameras == 1) {
        ROS_INFO("subscribing to: %s", param_io::param<std::string>(nh, "subscribers/" + camera_input_1 + "/topic", camera_input_1).c_str());
        subcam = imageNh.subscribe(param_io::param<std::string>(nh, "subscribers/" + camera_input_1 + "/topic", camera_input_1),
                              param_io::param<uint32_t>(nh, "subscribers/" + camera_input_1 + "/queue_size", 1), callback_monocular);
    } else if(params.state_options.num_cameras == 2) {
        ROS_INFO("subscribing to: %s", param_io::param<std::string>(nh, "subscribers/" + camera_input_1 + "/topic", camera_input_1).c_str());
        ROS_INFO("subscribing to: %s", param_io::param<std::string>(nh, "subscribers/" + camera_input_2 + "/topic", camera_input_2).c_str());
        sync.registerCallback(boost::bind(&callback_stereo, _1, _2));
    } else {
        ROS_ERROR("INVALID MAX CAMERAS SELECTED!!!");
        std::exit(EXIT_FAILURE);
    }

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Spin off to ROS
    ROS_INFO("Finish setting up subscribers.");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::AsyncSpinner imageSpinner(2, &imageCallbackQueue);
    imageSpinner.start();

    ros::waitForShutdown();

    // Final visualization
    viz->visualize_final();

    // Finally delete our system
    delete sys;
    delete viz;


    // Done!
    return EXIT_SUCCESS;


}


void callback_feet_contact(const signal_logger_msgs::UInt32Stamped::ConstPtr& msg0, const signal_logger_msgs::UInt32Stamped::ConstPtr& msg1, const signal_logger_msgs::UInt32Stamped::ConstPtr& msg2, const signal_logger_msgs::UInt32Stamped::ConstPtr& msg3) {

  double time = msg0->header.stamp.toSec();
  auto is_in_contact_foot0 = msg0->value;
  auto is_in_contact_foot1 = msg1->value;
  auto is_in_contact_foot2 = msg2->value;
  auto is_in_contact_foot3 = msg3->value;

  bool is_in_contact = (is_in_contact_foot0 != 0u) && (is_in_contact_foot1 != 0u) && (is_in_contact_foot2 != 0u) && (is_in_contact_foot3 != 0u);

  sys->feed_measurement_contact(time, is_in_contact);

}

void callback_inertial(const sensor_msgs::Imu::ConstPtr& msg) {

    // convert into correct format
    double timem = msg->header.stamp.toSec();
    Eigen::Vector3d wm, am;
    wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    // send it to our VIO system
    sys->feed_measurement_imu(timem, wm, am);
    viz->visualize_odometry(timem);

}



void callback_monocular(const sensor_msgs::ImageConstPtr& msg0) {


    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Fill our buffer if we have not
    if(img0_buffer.rows == 0) {
        time_buffer = cv_ptr->header.stamp.toSec();
        img0_buffer = cv_ptr->image.clone();
        return;
    }

    // send it to our VIO system
    sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);
    viz->visualize();

    // move buffer forward
    time_buffer = cv_ptr->header.stamp.toSec();
    img0_buffer = cv_ptr->image.clone();

}



void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1) {
    auto rT1 =  boost::posix_time::microsec_clock::local_time();
    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr0;
    try {
        cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr1;
    try {
        cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    // Fill our buffer if we have not
    if(img0_buffer.rows == 0 || img1_buffer.rows == 0) {
        time_buffer = cv_ptr0->header.stamp.toSec();
        img0_buffer = cv_ptr0->image.clone();
        time_buffer = cv_ptr1->header.stamp.toSec();
        img1_buffer = cv_ptr1->image.clone();
        return;
    }

    // send it to our VIO system
    sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);
    viz->visualize();

    // move buffer forward
    time_buffer = cv_ptr0->header.stamp.toSec();
    img0_buffer = cv_ptr0->image.clone();
    time_buffer = cv_ptr1->header.stamp.toSec();
    img1_buffer = cv_ptr1->image.clone();

    // Get timing statistics information
    auto rT2 =  boost::posix_time::microsec_clock::local_time();
    double time_track = (rT2-rT1).total_microseconds() * 1e-6;
    ROS_DEBUG_THROTTLE(3, "[TIME]: %.4f seconds for image callback (Debug is throttled: 3s)", time_track);

}


















