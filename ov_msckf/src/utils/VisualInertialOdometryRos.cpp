/*!
 * @file    VisualInertialOdometryRos.cpp
 * @author  Guoxiang Zhou (ANYbotics)
 * @date    02.11.20
 * @brief   Implementation of VisualInertialOdometryRos.h
 */

#include "VisualInertialOdometryRos.h"

#include <memory>

namespace ov_msckf {

VisualInertialOdometryRos::VisualInertialOdometryRos(ros::NodeHandle& nh, ros::NodeHandle& imageNh, ros::CallbackQueue& imageCallbackQueue): nh_(nh), imageNh_(imageNh) {
    // Use a separate callback queue for images input so that the images have a less chance to be dropped out.
    imageNh_.setCallbackQueue(&imageCallbackQueue);
    // Read parameters from ROS parameter server.
    params_ = parse_ros_nodehandler(nh_);
    bool success = read_ros_setup_parameters();
    if (!success){
        ROS_WARN("Cannot fetch ROS setup parameters. Will use the default values.");
    }

    sys_ = std::make_shared<VioManager>(params_);
    viz_ = std::make_unique<RosVisualizer>(nh_, sys_);

    if (params_.use_contact_for_initialization) {
        setup_contact_subscribers();
    }

    setup_mono_camera_callback_functions();

    setup_imu_subscribers();

    setup_image_subscribers();

    ROS_INFO("Finished setting up subscribers for the system.");
}

bool VisualInertialOdometryRos::read_ros_setup_parameters(){
    bool success = nh_.getParam("callback_switch_parameters/message_wait_time_out", message_wait_time_out_);
    success &= nh_.getParam("callback_switch_parameters/checking_frequency", checking_frequency_);
    success &= nh_.getParam("image_sync_time_diff_tolerance", image_sync_time_diff_tolerance_);
    return success;
}

void VisualInertialOdometryRos::setup_mono_camera_callback_functions(){
    camera_id_to_use_vec_ = params_.state_options.camera_id_to_use_vec;
    for (auto camera_id: camera_id_to_use_vec_){
        mono_camera_callback_map_[camera_id] = boost::bind(&VisualInertialOdometryRos::callback_monocular, this, _1, camera_id);
    }
}

void VisualInertialOdometryRos::setup_image_subscribers() {
    const auto num_of_cameras_to_use = camera_id_to_use_vec_.size();
    if(num_of_cameras_to_use == 1) {
        setup_camera_subscriber(camera_id_to_use_vec_.back());
    } else if(num_of_cameras_to_use == 2) {
        // todo (GZ): store the subscribers and camera topic key names into a vector so that we can automate it in the future.
        // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491.
        first_image_sub_.subscribe(imageNh_, param_io::param<std::string>(nh_, "subscribers/" + camera_input_0_ + "/topic", camera_input_0_),
                                   param_io::param<uint32_t>(nh_, "subscribers/" + camera_input_0_ + "/queue_size", 1));
        second_image_sub_.subscribe(imageNh_,param_io::param<std::string>(nh_, "subscribers/" + camera_input_1_ + "/topic", camera_input_1_),
                                    param_io::param<uint32_t>(nh_, "subscribers/" + camera_input_1_ + "/queue_size", 1));
        image_pair_sync_ptr_ = std::make_unique<message_filters::Synchronizer<image_pair_sync_pol>>(image_pair_sync_pol(5), first_image_sub_, second_image_sub_);
        ROS_INFO("Image subscribers: subscribed to: %s", param_io::param<std::string>(nh_, "subscribers/" + camera_input_0_ + "/topic", camera_input_0_).c_str());
        ROS_INFO("Image subscribers: subscribed to: %s", param_io::param<std::string>(nh_, "subscribers/" + camera_input_1_ + "/topic", camera_input_1_).c_str());
        image_pair_sync_ptr_->registerCallback(boost::bind(&VisualInertialOdometryRos::callback_stereo, this, _1, _2));
    } else {
        ROS_ERROR("The number of cameras used for the system is invalid. System exiting...");
        std::exit(EXIT_FAILURE);
    }
}

void VisualInertialOdometryRos::setup_camera_subscriber(int camera_id) {
    const std::string camera_input_string_key = camera_id == 0 ? camera_input_0_ : camera_input_1_;
    ROS_INFO("Image subscribers: subscribed to: %s", param_io::param<std::string>(nh_, "subscribers/" + camera_input_string_key + "/topic", camera_input_string_key).c_str());
    mono_img_sub_ = imageNh_.subscribe(param_io::param<std::string>(nh_, "subscribers/" + camera_input_string_key + "/topic", camera_input_string_key),
                                       param_io::param<uint32_t>(nh_, "subscribers/" + camera_input_string_key + "/queue_size", 1),
                                       mono_camera_callback_map_[camera_id]);
}

void VisualInertialOdometryRos::setup_imu_subscribers() {
    imu_subscriber_ = nh_.subscribe(param_io::param<std::string>(nh_, "subscribers/" + imu_input_ + "/topic", imu_input_),
                                    param_io::param<uint32_t>(nh_, "subscribers/" + imu_input_ + "/queue_size", 1), &VisualInertialOdometryRos::callback_inertial, this);
}

void VisualInertialOdometryRos::setup_contact_subscribers(){
    const std::string contact_force_lf_foot = "contact_force_lf_foot";
    const std::string contact_force_rf_foot = "contact_force_rf_foot";
    const std::string contact_force_lh_foot = "contact_force_lh_foot";
    const std::string contact_force_rh_foot = "contact_force_rh_foot";

    // Create feet contact subscriber.
    foot_lf_sub_.subscribe(nh_,param_io::param<std::string>(nh_, "subscribers/" + contact_force_lf_foot + "/topic", contact_force_lf_foot),
                                                                               param_io::param<uint32_t>(nh_, "subscribers/" + contact_force_lf_foot + "/queue_size", 1));
    foot_lh_sub_.subscribe(nh_,param_io::param<std::string>(nh_, "subscribers/" + contact_force_lh_foot + "/topic", contact_force_lh_foot),
                                                                               param_io::param<uint32_t>(nh_, "subscribers/" + contact_force_lh_foot + "/queue_size", 1));
    foot_rf_sub_.subscribe(nh_,param_io::param<std::string>(nh_, "subscribers/" + contact_force_rf_foot + "/topic", contact_force_rf_foot),
                                                                               param_io::param<uint32_t>(nh_, "subscribers/" + contact_force_rf_foot + "/queue_size", 1));
    foot_rh_sub_.subscribe(nh_,param_io::param<std::string>(nh_, "subscribers/" + contact_force_rh_foot + "/topic", contact_force_rh_foot),
                                                                               param_io::param<uint32_t>(nh_, "subscribers/" + contact_force_rh_foot + "/queue_size", 1));
    feet_contact_sync_ptr_ = std::make_unique<message_filters::Synchronizer<feet_contact_sync_pol>>(feet_contact_sync_pol(9999), foot_lf_sub_,foot_lh_sub_, foot_rf_sub_, foot_rh_sub_);
    feet_contact_sync_ptr_->registerCallback(boost::bind(&VisualInertialOdometryRos::callback_feet_contact, this, _1, _2, _3, _4));
}

void VisualInertialOdometryRos::callback_switch(){
    while(ros::ok()) {
        auto image_0_msg = ros::topic::waitForMessage<sensor_msgs::Image>(param_io::param<std::string>(nh_, "subscribers/" + camera_input_0_ + "/topic", camera_input_0_),
            imageNh_, ros::Duration(message_wait_time_out_));
        auto image_1_msg = ros::topic::waitForMessage<sensor_msgs::Image>(param_io::param<std::string>(nh_, "subscribers/" + camera_input_1_ + "/topic", camera_input_1_),
            imageNh_, ros::Duration(message_wait_time_out_));

        if (image_0_msg && image_1_msg){
            // Shutdown the mono image subscriber.
            if(mono_img_sub_ != nullptr) {
                mono_img_sub_.shutdown();
            }

            // Start the synchronized image subscribers.
            if(first_image_sub_.getSubscriber() == nullptr)
            {
                first_image_sub_.subscribe(imageNh_, param_io::param<std::string>(nh_, "subscribers/" + camera_input_0_ + "/topic", camera_input_0_),
                                           param_io::param<uint32_t>(nh_, "subscribers/" + camera_input_0_ + "/queue_size", 1));
            }
            if(second_image_sub_.getSubscriber() == nullptr)
            {
                second_image_sub_.subscribe(imageNh_,param_io::param<std::string>(nh_, "subscribers/" + camera_input_1_ + "/topic", camera_input_1_),
                                            param_io::param<uint32_t>(nh_, "subscribers/" + camera_input_1_ + "/queue_size", 1));
            }
        }else if (image_0_msg && !image_1_msg){
            // Shutdown the synchronized image subscribers.
            if (first_image_sub_.getSubscriber() != nullptr)
            {
                first_image_sub_.unsubscribe();
            }
            if (second_image_sub_.getSubscriber() != nullptr)
            {
                second_image_sub_.unsubscribe();
            }
            // Start the mono image subscriber.
            if (mono_img_sub_ == nullptr ||
            mono_img_sub_.getTopic() != param_io::param<std::string>(nh_, "subscribers/" + camera_input_0_ + "/topic", camera_input_0_)) {
                // Try subscribing to the first image topic.
                mono_img_sub_ = imageNh_.subscribe(param_io::param<std::string>(nh_,"subscribers/" + camera_input_0_ + "/topic", camera_input_0_),
                                                   param_io::param<uint32_t>(nh_, "subscribers/" + camera_input_0_ + "/queue_size", 1),
                                                   mono_camera_callback_map_[0]);
            }

            ROS_WARN_STREAM_ONCE("Cannot get image from topic " << param_io::param<std::string>(nh_, "subscribers/" + camera_input_1_ + "/topic", camera_input_1_) << ". Started monocular visual-inertial odometry.");
        }else if (!image_0_msg && image_1_msg){
            if (first_image_sub_.getSubscriber() != nullptr)
            {
                first_image_sub_.unsubscribe();
            }
            if (second_image_sub_.getSubscriber() != nullptr)
            {
                second_image_sub_.unsubscribe();
            }

            if (mono_img_sub_ == nullptr ||
                mono_img_sub_.getTopic() != param_io::param<std::string>(nh_, "subscribers/" + camera_input_1_ + "/topic",
                                                                         camera_input_1_))  {
                // Try subscribing to the second image topic.
                mono_img_sub_ = imageNh_.subscribe(param_io::param<std::string>(nh_,"subscribers/" + camera_input_1_ + "/topic",camera_input_1_),
                                                   param_io::param<uint32_t>(nh_, "subscribers/" + camera_input_1_ + "/queue_size", 1),
                                                   mono_camera_callback_map_[1]);
            }

            ROS_WARN_STREAM_ONCE("Cannot get image from topic " << param_io::param<std::string>(nh_, "subscribers/" + camera_input_0_ + "/topic", camera_input_0_) << ". Started monocular visual-inertial odometry.");
        }else{
            ROS_WARN_STREAM_THROTTLE(3, "No image for visual-inertial odometry is currently available (throttled: 3s)");
        }
        ros::Duration(1/checking_frequency_).sleep();
    }
}

void VisualInertialOdometryRos::callback_feet_contact(const signal_logger_msgs::UInt32Stamped::ConstPtr& msg0, const signal_logger_msgs::UInt32Stamped::ConstPtr& msg1,
                                                      const signal_logger_msgs::UInt32Stamped::ConstPtr& msg2, const signal_logger_msgs::UInt32Stamped::ConstPtr& msg3) {
    const double time = msg0->header.stamp.toSec();
    const auto is_in_contact_foot0 = msg0->value;
    const auto is_in_contact_foot1 = msg1->value;
    const auto is_in_contact_foot2 = msg2->value;
    const auto is_in_contact_foot3 = msg3->value;

    const bool is_in_contact = (is_in_contact_foot0 != 0u) && (is_in_contact_foot1 != 0u) && (is_in_contact_foot2 != 0u) && (is_in_contact_foot3 != 0u);
    sys_->feed_measurement_contact(time, is_in_contact);
}

void VisualInertialOdometryRos::callback_inertial(const sensor_msgs::Imu::ConstPtr& msg) {
    // Convert the IMU message into a proper format.
    const double imu_msg_time = msg->header.stamp.toSec();
    Eigen::Vector3d wm, am;
    wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    // Send the IMU measurement to the VIO system.
    sys_->feed_measurement_imu(imu_msg_time, wm, am);
    viz_->visualize_odometry(imu_msg_time);
}

void VisualInertialOdometryRos::callback_monocular(const sensor_msgs::ImageConstPtr &msg, int camera_id) {
    // todo(GZ): image callbacks should be very fast. Need check!
    // Get the image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (img_mat_buffer_map_[camera_id].rows == 0) {
        image_timestamp_buffer_map_[camera_id] = cv_ptr->header.stamp.toSec();
        img_mat_buffer_map_[camera_id] = cv_ptr->image.clone();
        return;
    }

    // Send it to our VIO system.
    sys_->feed_measurement_monocular(image_timestamp_buffer_map_[camera_id] , img_mat_buffer_map_[camera_id], camera_id);
    viz_->visualize();

    // Move buffer forward.
    image_timestamp_buffer_map_[camera_id]  = cv_ptr->header.stamp.toSec();
    img_mat_buffer_map_[camera_id] = cv_ptr->image.clone();

}

void VisualInertialOdometryRos::callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1) {

    auto rT1 =  boost::posix_time::microsec_clock::local_time();
    // Get the first image.
    cv_bridge::CvImageConstPtr cv_ptr0;
    try {
        cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Get the second image.
    cv_bridge::CvImageConstPtr cv_ptr1;
    try {
        cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // todo (GZ): make the logic below better. iterate the camera ids in this camera_id_to_use_vec_ instead of using 0 and 1
    // Fill our buffer if we have not.
    if(img_mat_buffer_map_[0].rows == 0 || img_mat_buffer_map_[1].rows == 0) {
        image_timestamp_buffer_map_[0] = cv_ptr0->header.stamp.toSec();
        img_mat_buffer_map_[0] = cv_ptr0->image.clone();
        image_timestamp_buffer_map_[1] = cv_ptr1->header.stamp.toSec();
        img_mat_buffer_map_[1] = cv_ptr1->image.clone();
        return;
    }

    // Send it to our VIO system.
    sys_->feed_measurement_stereo(image_timestamp_buffer_map_, img_mat_buffer_map_, 0, 1, image_sync_time_diff_tolerance_);
    viz_->visualize();

    // Move buffer forward.
    image_timestamp_buffer_map_[0] = cv_ptr0->header.stamp.toSec();
    img_mat_buffer_map_[0] = cv_ptr0->image.clone();
    image_timestamp_buffer_map_[1] = cv_ptr1->header.stamp.toSec();
    img_mat_buffer_map_[1] = cv_ptr1->image.clone();

    // Get timing statistics information.
    auto rT2 =  boost::posix_time::microsec_clock::local_time();
    double time_track = (rT2-rT1).total_microseconds() * 1e-6;
    ROS_DEBUG_THROTTLE(3, "%.4f seconds used for stereo image callback (Throttled: 3s)", time_track);
}
}  // namespace ov_msckf
