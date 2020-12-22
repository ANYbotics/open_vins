/*!
 * @file    VisualInertialOdometryRos.h
 * @author  Guoxiang Zhou (ANYbotics)
 * @date    02.11.20
 * @brief   Set up ROS interface for visual inertial odometry
 */

#pragma once

#include "../core/RosVisualizer.h"
#include "../core/VioManager.h"
#include "../core/VioManagerOptions.h"
#include "../utils/parse_ros.h"

// ROS
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <kindr/Core>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>

#include <boost/date_time/microsec_time_clock.hpp>

#include <param_io/get_param.hpp>

#include <ov_core/utils/dataset_reader.h>

#include <signal_logger_msgs/UInt32Stamped.h>

namespace ov_msckf {

class VisualInertialOdometryRos{

public:
    explicit VisualInertialOdometryRos(ros::NodeHandle& nh, ros::NodeHandle& imageNh, ros::CallbackQueue& imageCallbackQueue);
    // todo (GZ): extend this function to support multiple cameras (2+)?
    /**
     * @brief Check whether the sensor is streaming images and set up the corresponding callbacks for processing.
     */
    void callback_switch();
    /**
     * @brief The reset service callback.
     * @param request The service message from the client.
     * @param response The service message return from the serve.
     * @return True if the service call is successful.
     */
    bool reset_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    /**
     * @brief Whether or not to start the thread to handle sensor failure.
     * @return True if a thread is required to spawn, false otherwise.
     */
    bool enable_sensor_failure_handler_thread();
    /**
     * @brief Publish the notification when the system is reset.
     */
    void publish_reset_notification() const;
    VioManagerOptions params_;
    std::shared_ptr<RosVisualizer> viz_;

private:
    /**
     * @brief ROS callback function of IMU message.
     * @param msg Input IMU message.
     */
    void callback_inertial(const sensor_msgs::Imu::ConstPtr& msg);
    /**
     * @brief Monocular image callback function.
     * @param msg         Input Image message.
     * @param camera_id   The camera id of the image.
     */
    void callback_monocular(const sensor_msgs::ImageConstPtr &msg, int camera_id = 0);
    /**
     * @brief Synchronized image-pair callback function.
     * @param msg0 The first input image message.
     * @param msg1 The second input image message.
     */
    void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1);
    /**
     * @brief Synchronized contact callback function
     * @param msg0 The contact message of the left front foot.
     * @param msg1 The contact message of the left hind foot.
     * @param msg2 The contact message of the right front foot.
     * @param msg3 The contact message of the right hind foot.
     */
    void callback_feet_contact(const signal_logger_msgs::UInt32Stamped::ConstPtr& msg0, const signal_logger_msgs::UInt32Stamped::ConstPtr& msg1,
                             const signal_logger_msgs::UInt32Stamped::ConstPtr& msg2, const signal_logger_msgs::UInt32Stamped::ConstPtr& msg3);

    /**
     * @brief Set up subscribers for contact ROS message.
     */
    void setup_contact_subscribers();
    /**
     * @brief Set up subscribers for IMU ROS message.
     */
    void setup_imu_subscribers();
    /**
     * @brief Set up the system subscribers for image ROS message.
     */
    void setup_image_subscribers();
    /**
     * @brief Set up the subscriber for a single camera.
     * @param camera_id The id of the camera.
     */
    void setup_camera_subscriber(int camera_id);
    /**
     * @brief Assign callback functions for mono cameras.
     */
    void setup_mono_camera_callback_functions();
    /**
     * @brief Get the rigid body transform from imu to robot base frame from TF tree.
     * @return True if the transform can be retrieved, false otherwise.
     */
    bool get_kindr_rigid_body_transform_from_imu_to_robot_frame();

    bool read_vio_setup_parameters();

    // Define message_filters synchronization policies for image topics.
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> image_pair_sync_pol;
    // Define message_filters synchronization policies for contact topics.
    typedef message_filters::sync_policies::ExactTime<signal_logger_msgs::UInt32Stamped, signal_logger_msgs::UInt32Stamped,
    signal_logger_msgs::UInt32Stamped, signal_logger_msgs::UInt32Stamped> feet_contact_sync_pol;

    std::shared_ptr<VioManager> vio_manager_;

    // Buffer data.
    std::map<unsigned int, double> image_timestamp_buffer_map_;
    std::map<unsigned int, cv::Mat> img_mat_buffer_map_;

    // ROS nodehandles.
    ros::NodeHandle nh_;
    // Separate ROS nodehandle handle for images to set up the separate callback queue.
    ros::NodeHandle imageNh_;

    // Image subscribers.
    ros::Subscriber mono_img_sub_;
    message_filters::Subscriber<sensor_msgs::Image> first_image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> second_image_sub_;

    // Feet contact subscribers.
    message_filters::Subscriber<signal_logger_msgs::UInt32Stamped> foot_lf_sub_;
    message_filters::Subscriber<signal_logger_msgs::UInt32Stamped> foot_lh_sub_;
    message_filters::Subscriber<signal_logger_msgs::UInt32Stamped> foot_rf_sub_;
    message_filters::Subscriber<signal_logger_msgs::UInt32Stamped> foot_rh_sub_;

    // IMU subscriber.
    ros::Subscriber imu_subscriber_;

    std::map<unsigned int, boost::function<void(const boost::shared_ptr<sensor_msgs::Image const>&)>> mono_camera_callback_map_;

    // Reset server.
    ros::ServiceServer reset_server_;
    // Reset Notification Publisher.
    ros::Publisher reset_notification_publisher_;

    std::unique_ptr<message_filters::Synchronizer<image_pair_sync_pol>> image_pair_sync_ptr_;
    std::unique_ptr<message_filters::Synchronizer<feet_contact_sync_pol>> feet_contact_sync_ptr_;

    const std::string camera_input_0_ = "image_topic_0";
    const std::string camera_input_1_ = "image_topic_1";
    const std::string imu_input_ = "imu_sensor";
    
    // Parameters used in callback_switch.
    double message_wait_time_out_ = 1.0;
    double checking_frequency_ = 1.0;
    // The maximum allowed timestamp difference among a synced image-pair. Unit: second.
    double image_sync_time_diff_tolerance_ = 0.004;
    // The vector of the camera ids that will be used for VIO.
    std::vector<int> camera_id_to_use_vec_;
    // Whether or not to handle sensor failure.
    bool handle_sensor_failure_ = false;
    // Whether to transform the IMU measurements from its own frame to robot (base) frame.
    bool transform_imu_into_robot_frame_ = false;

    /// TF buffer.
    tf2_ros::Buffer tf_buffer_;

    tf2_ros::TransformListener tf_listener_;
    /// The rigid body transformation from IMU to robot base frame.
    kindr::HomogeneousTransformationPosition3RotationQuaternionD T_RI_;

    string imu_frame_id_ = "imu_link";
    string robot_frame_id_ = "base";
    double tf_timeout_imu_robot_ = 1.0;
};
}  // namespace ov_msckf
