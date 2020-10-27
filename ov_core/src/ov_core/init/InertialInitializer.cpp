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
#include "InertialInitializer.h"
#include <ros/console.h>


using namespace ov_core;


void InertialInitializer::feed_imu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am) {

    // Create our imu data object
    IMUDATA data;
    data.timestamp = timestamp;
    data.wm = wm;
    data.am = am;

    {
        std::lock_guard<std::mutex> lock(imu_data_mutex_);
        // Append it to our vector
        imu_data.emplace_back(data);

        // Delete all measurements older than 2*_initialization_window_length
        auto it0 = imu_data.begin();
        while (it0 != imu_data.end() && it0->timestamp < timestamp - (2 * _initialization_window_length)) {
            it0 = imu_data.erase(it0);
        }
        ROS_DEBUG_STREAM_THROTTLE(3, "The number of IMU measurements collected for initialization is: " << imu_data.size() << " (Throttled: 3s).");
    }

}

void InertialInitializer::feed_contact(double timestamp, bool is_in_contact) {

    FeetContact data;
    data.timestamp = timestamp;
    data.is_in_contact = is_in_contact;
    {
        std::lock_guard<std::mutex> lock(contact_data_mutex_);
        contact_data.emplace_back(data);
        // Delete all measurements older than 2*_initialization_window_length
        auto it0 = contact_data.begin();
        while (it0 != contact_data.end() && it0->timestamp < timestamp - (2 * _initialization_window_length)) {
            it0 = contact_data.erase(it0);
        }
    }
}

bool InertialInitializer::obtain_initialization_window(double & window_start, double & window_end, bool consider_contact, const double initialization_window_length)
{
    if(imu_data.empty() || (imu_data.back().timestamp - imu_data.front().timestamp <= initialization_window_length)) {
        ROS_WARN_THROTTLE(1, "Not enough IMU measurements in the initialization window. (Warning is throttled: 1s)");
        return false;
    }
    if(consider_contact) {
        if ( contact_data.empty() || (contact_data.back().timestamp - contact_data.front().timestamp <= initialization_window_length)){
            ROS_WARN_THROTTLE(1, "Not enough contact measurements in the initialization window. (Warning is throttled: 1s)");
            return false;
        }
        // Check consecutive contact condition
        int contact_data_size = contact_data.size();
        int contact_data_latest_index = contact_data_size - 1;
        double initialization_start_timestamp = contact_data[contact_data_latest_index].timestamp - initialization_window_length;
        bool is_in_consecutive_contact = true;
        // todo(GZ): Notice here the number of checked contact data might be very small due to the message dropout.
        while (contact_data[contact_data_latest_index].timestamp >= initialization_start_timestamp) {
            is_in_consecutive_contact &= contact_data[contact_data_latest_index--].is_in_contact;
            if (!is_in_consecutive_contact) {
                ROS_WARN_THROTTLE(1, "The agent is not in contact in the initialization window. (Warning is throttled: 1s)");
                return false;
            }
        }
        window_start = contact_data[contact_data_latest_index+1].timestamp;
        window_end = contact_data[contact_data_size - 1].timestamp;
    }
    else
    {
        window_end = imu_data.back().timestamp;
        window_start = window_end - initialization_window_length;
    }
    return true;
}

bool InertialInitializer::fetch_imu_data_for_initialization(std::vector<IMUDATA>& valid_imu_measurements_window, double initialization_window_length, int num_valid_imu_measurements_threshold, bool consider_contact){
    std::lock_guard<std::mutex> lock1(imu_data_mutex_);
    std::lock_guard<std::mutex> lock2(contact_data_mutex_);

    double valid_window_start_time = 0;
    double valid_window_end_time = 0;
    if (!obtain_initialization_window(valid_window_start_time, valid_window_end_time, consider_contact, initialization_window_length))
    {
        return false;
    }
    // Collect a window of IMU readings in reverse time order from the chosen period where the agent is in contact with the ground.
    int imu_measurements_counter = 0;
    for(auto iter = imu_data.rbegin(); iter!=imu_data.rend(); iter++) {
        if(iter->timestamp > valid_window_start_time && iter->timestamp < valid_window_end_time) {
            valid_imu_measurements_window.push_back(*iter);
            if (imu_measurements_counter++ >= num_valid_imu_measurements_threshold) {
                return true;
            }
        }
    }

    // Return false if there is not enough "valid" IMU measurements.
    ROS_WARN_THROTTLE(1, "Not enough valid IMU measurements %d/%d in the initialization window (Warning is throttled: 1s)", imu_measurements_counter, num_valid_imu_measurements_threshold);
    if (!valid_imu_measurements_window.empty()) {
        ROS_WARN_STREAM_THROTTLE(1,
                                 "The initialization window time at the end minus the timestamp of the latest selected IMU measurement is "
                                     << valid_window_end_time - valid_imu_measurements_window.begin()->timestamp
                                     << " (Warning is throttled: 1s).");
    }
    return false;
}

bool InertialInitializer::initialize_with_imu(double &time0, Eigen::Matrix<double,4,1> &q_GtoI0, Eigen::Matrix<double,3,1> &b_w0,
                                              Eigen::Matrix<double,3,1> &v_I0inG, Eigen::Matrix<double,3,1> &b_a0, Eigen::Matrix<double,3,1> &p_I0inG, bool consider_contact) {

    std::vector<IMUDATA> initialization_imu_window;

    if (!fetch_imu_data_for_initialization(initialization_imu_window, _initialization_window_length, _imu_init_num_of_imu_measurements, consider_contact)) {
        return false;
    }

    // Sum up our current accelerations and velocities
    Eigen::Vector3d linsum = Eigen::Vector3d::Zero();
    Eigen::Vector3d angsum = Eigen::Vector3d::Zero();

    for(const auto& imu_data : initialization_imu_window) {
        linsum += imu_data.am;
        angsum += imu_data.wm;
    }

    // Calculate the mean of the linear acceleration and angular velocity
    Eigen::Vector3d linavg = Eigen::Vector3d::Zero();
    Eigen::Vector3d angavg = Eigen::Vector3d::Zero();
    linavg = linsum/ initialization_imu_window.size();
    angavg = angsum/ initialization_imu_window.size();

    // Check IMU acceleration variance.
    double a_var = 0;
    for(IMUDATA data : initialization_imu_window) {
      a_var += (data.am-linavg).dot(data.am-linavg);
    }
    a_var = std::sqrt(a_var/(initialization_imu_window.size()-1));

    ROS_DEBUG_THROTTLE(1, "IMU acceleration variance in the initialization time window: %.4f (Debugging is throttled: 1s)", a_var);

    // If acceleration variance is above the threshold, this means the agent is not standstill. Return false.
    if(a_var > _imu_init_threshold ) {
      ROS_WARN_THROTTLE(1, "Too much IMU acceleration variance detected in the initialization time window, above the threshold %.4f > %.4f (Warning is throttled: 1s)", a_var, _imu_init_threshold);
      return false;
    }

    // Get z axis, which alines with -g (z_in_G=0,0,1)
    Eigen::Vector3d z_axis = linavg/linavg.norm();

    // Create an x_axis
    Eigen::Vector3d e_1(1,0,0);

    // Make x_axis perpendicular to z
    Eigen::Vector3d x_axis = e_1-z_axis*z_axis.transpose()*e_1;
    x_axis= x_axis/x_axis.norm();

    // Get z from the cross product of these two
    Eigen::Matrix<double,3,1> y_axis = skew_x(z_axis)*x_axis;

    // From these axes get rotation
    Eigen::Matrix<double,3,3> Ro;
    Ro.block(0,0,3,1) = x_axis;
    Ro.block(0,1,3,1) = y_axis;
    Ro.block(0,2,3,1) = z_axis;

    // Create our state variables
    Eigen::Matrix<double,4,1> q_GtoI = rot_2_quat(Ro);

    // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
    Eigen::Matrix<double,3,1> bg = angavg;
    Eigen::Matrix<double,3,1> ba = linavg - quat_2_Rot(q_GtoI)*_gravity;

    // Set our state variables
    time0 = initialization_imu_window.back().timestamp;
    q_GtoI0 = q_GtoI;
    b_w0 = bg;
    v_I0inG = Eigen::Matrix<double,3,1>::Zero();
    b_a0 = ba;
    p_I0inG = Eigen::Matrix<double,3,1>::Zero();

    // Done!!!
    return true;


}




