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
#ifndef OV_MSCKF_VIOMANAGER_H
#define OV_MSCKF_VIOMANAGER_H


#include <string>
#include <algorithm>
#include <atomic>
#include <fstream>
#include <Eigen/StdVector>
#include <boost/filesystem.hpp>
#include <opencv2/core/mat.hpp>

#include <ov_core/track/TrackAruco.h>
#include <ov_core/track/TrackDescriptor.h>
#include <ov_core/track/TrackKLT.h>
#include <ov_core/track/TrackSIM.h>
#include <ov_core/init/InertialInitializer.h>
#include <ov_core/types/LandmarkRepresentation.h>
#include <ov_core/types/Landmark.h>

#include <tbb/task_group.h>
#include <tbb/task_scheduler_init.h>

#include <ros/service_client.h>

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "update/UpdaterZeroVelocity.h"

#include "VioManagerOptions.h"


namespace ov_msckf {



    /**
     * @brief Core class that manages the entire system
     *
     * This class contains the state and other algorithms needed for the MSCKF to work.
     * We feed in measurements into this class and send them to their respective algorithms.
     * If we have measurements to propagate or update with, this class will call on our state to do that.
     */
    class VioManager {


    public:


        /**
         * @brief Class constructor, will load all configuration variables
         * @param params_ Parameters loaded from either ROS or CMDLINE
         */
        VioManager(VioManagerOptions& params_);

        /**
         * @brief Class constructor, will load all configuration variables and set up a ros::ServiceClient.
         * @param params_ Parameters loaded from either ROS or CMDLINE.
         * @param reset_client The ros service client used to call the reset service server.
         */
        VioManager(VioManagerOptions& params_, ros::ServiceClient reset_client);


        /**
         * @brief Feed function for inertial data
         * @param timestamp Time of the inertial measurement
         * @param wm Angular velocity
         * @param am Linear acceleration
         */
        void feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am);

        /**
         * @brief Feed function for feet contact data
         * @param timestamp Time of the feet contact measurement
         * @param is_in_contact Whether the agent is in contact or not
         */
        void feed_measurement_contact(double timestamp, bool is_in_contact);


        /**
         * @brief Feed function for a single camera
         * @param timestamp Time that this image was collected
         * @param img0 Grayscale image
         * @param cam_id Unique id of what camera the image is from
         */
        void feed_measurement_monocular(double timestamp, cv::Mat& img0, size_t cam_id);

        /**
         * @brief Feed function for stereo camera pair with image synchronization check
         * @param image_timestamp_buffer_map The image timestamp map (camera id, timestamp)
         * @param image_mat_buffer_map The image data map (camera id, image data)
         * @param cam_id0 Unique id of the first camera
         * @param cam_id1 Unique id of the second camera
         * @param image_sync_time_diff_tolerance The maximum allowed timestamp difference among a synced image-pair. Unit: second.
         */
        void feed_measurement_stereo(std::map<unsigned int, double> &image_timestamp_buffer_map,
                                     std::map<unsigned int, cv::Mat> &image_mat_buffer_map,
                                     size_t cam_id0,
                                     size_t cam_id1,
                                     const double image_sync_time_diff_tolerance);
        /**
         * @brief Feed function for stereo camera pair
         * @param timestamp Time that this image was collected
         * @param img0 Grayscale image
         * @param img1 Grayscale image
         * @param cam_id0 Unique id of what camera the image is from
         * @param cam_id1 Unique id of what camera the image is from
         */
        void feed_measurement_stereo(double timestamp, cv::Mat& img0, cv::Mat& img1, size_t cam_id0, size_t cam_id1);

        /**
         * @brief Feed function for a synchronized simulated cameras
         * @param timestamp Time that this image was collected
         * @param camids Camera ids that we have simulated measurements for
         * @param feats Raw uv simulated measurements
         */
        void feed_measurement_simulation(double timestamp, const std::vector<int> &camids, const std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> &feats);

        /**
         * @brief Given a state, this will initialize our IMU state.
         * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
         */
        void initialize_with_gt(Eigen::Matrix<double,17,1> imustate) {

            // Initialize the system
            state->_imu->set_value(imustate.block(1,0,16,1));
            state->_imu->set_fej(imustate.block(1,0,16,1));
            state->_timestamp = imustate(0,0);
            startup_time = imustate(0,0);
            is_initialized_vio = true;

            // Cleanup any features older then the initialization time
            trackFEATS->get_feature_database()->cleanup_measurements(state->_timestamp);
            if(trackARUCO != nullptr) {
                trackARUCO->get_feature_database()->cleanup_measurements(state->_timestamp);
            }

            // Print what we init'ed with
            printf(GREEN "[INIT]: INITIALIZED FROM GROUNDTRUTH FILE!!!!!\n" RESET);
            printf(GREEN "[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\n" RESET,state->_imu->quat()(0),state->_imu->quat()(1),state->_imu->quat()(2),state->_imu->quat()(3));
            printf(GREEN "[INIT]: bias gyro = %.4f, %.4f, %.4f\n" RESET,state->_imu->bias_g()(0),state->_imu->bias_g()(1),state->_imu->bias_g()(2));
            printf(GREEN "[INIT]: velocity = %.4f, %.4f, %.4f\n" RESET,state->_imu->vel()(0),state->_imu->vel()(1),state->_imu->vel()(2));
            printf(GREEN "[INIT]: bias accel = %.4f, %.4f, %.4f\n" RESET,state->_imu->bias_a()(0),state->_imu->bias_a()(1),state->_imu->bias_a()(2));
            printf(GREEN "[INIT]: position = %.4f, %.4f, %.4f\n" RESET,state->_imu->pos()(0),state->_imu->pos()(1),state->_imu->pos()(2));

        }


        /// If we are initialized or not
        bool initialized() {
            return is_initialized_vio;
        }

        /**
         * @brief Set the system initialization state.
         * @param value The value used to set the system initialization state.
         */
        void set_initialization_flag(bool value){
            if (is_initialized_vio != value) {
                is_initialized_vio = value;
            }
        }

        /**
         * @brief Reset the VIO system. It does the following:
         * 1. Set the system initialization state to false and so that the system will restart initialization.
         * 2. Clear the whole saved feature database.
         * 3. Set the flag to notify the Visualizer to clear the VIO path poses.
         */
        void reset(){
            // Thread-safe.
            set_initialization_flag(false);
            trackFEATS->reset();
            // Thread-safe.
            set_clear_vio_path_in_visualization(true);
        }

        /// Timestamp that the system was initialized at
        double initialized_time() {
            return startup_time;
        }

        /// Accessor to get the current state
        State* get_state() {
            return state;
        }

        /// Accessor to get the current propagator
        Propagator* get_propagator() {
            return propagator;
        }

        /// Get feature tracker
        TrackBase* get_track_feat() {
            return trackFEATS;
        }

        /// Get aruco feature tracker
        TrackBase* get_track_aruco() {
            return trackARUCO;
        }

        /// Returns 3d features used in the last update in global frame
        std::vector<Eigen::Vector3d> get_good_features_MSCKF() {
            return good_features_MSCKF;
        }

        /// Returns 3d SLAM features in the global frame
        std::vector<Eigen::Vector3d> get_features_SLAM() {
            std::vector<Eigen::Vector3d> slam_feats;
            for (auto &f : state->_features_SLAM) {
                if((int)f.first <= state->_options.max_aruco_features) continue;
                if(LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
                    // Assert that we have an anchor pose for this feature
                    assert(f.second->_anchor_cam_id!=-1);
                    // Get calibration for our anchor camera
                    Eigen::Matrix<double, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->Rot();
                    Eigen::Matrix<double, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->pos();
                    // Anchor pose orientation and position
                    Eigen::Matrix<double,3,3> R_GtoI = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->Rot();
                    Eigen::Matrix<double,3,1> p_IinG = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->pos();
                    // Feature in the global frame
                    slam_feats.push_back(R_GtoI.transpose() * R_ItoC.transpose()*(f.second->get_xyz(false) - p_IinC) + p_IinG);
                } else {
                    slam_feats.push_back(f.second->get_xyz(false));
                }
            }
            return slam_feats;
        }

        /// Returns 3d ARUCO features in the global frame
        std::vector<Eigen::Vector3d> get_features_ARUCO() {
            std::vector<Eigen::Vector3d> aruco_feats;
            for (auto &f : state->_features_SLAM) {
                if((int)f.first > state->_options.max_aruco_features) continue;
                if(LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
                    // Assert that we have an anchor pose for this feature
                    assert(f.second->_anchor_cam_id!=-1);
                    // Get calibration for our anchor camera
                    Eigen::Matrix<double, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->Rot();
                    Eigen::Matrix<double, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->pos();
                    // Anchor pose orientation and position
                    Eigen::Matrix<double,3,3> R_GtoI = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->Rot();
                    Eigen::Matrix<double,3,1> p_IinG = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->pos();
                    // Feature in the global frame
                    aruco_feats.push_back(R_GtoI.transpose() * R_ItoC.transpose()*(f.second->get_xyz(false) - p_IinC) + p_IinG);
                } else {
                    aruco_feats.push_back(f.second->get_xyz(false));
                }
            }
            return aruco_feats;
        }

        /// Return true if we did a zero velocity update
        bool did_zero_velocity_update(){
            return did_zupt_update;
        }

        /// Return the zero velocity update image
        cv::Mat get_zero_velocity_update_image() {
            return zupt_image;
        }

        /// Returns the last timestamp we have marginalized (true if we have a state)
        bool hist_last_marg_state(double &timestamp, Eigen::Matrix<double,7,1> &stateinG) {
            if(hist_last_marginalized_time != -1) {
                timestamp = hist_last_marginalized_time;
                stateinG = hist_stateinG.at(hist_last_marginalized_time);
                return true;
            } else {
                timestamp = -1;
                stateinG.setZero();
                return false;
            }
        }

        /// Returns historical feature positions, and measurements times and uvs used to get its estimate.
        void hist_get_features(std::unordered_map<size_t,Eigen::Vector3d> &feat_posinG,
                               std::unordered_map<size_t, std::unordered_map<size_t, std::vector<Eigen::VectorXf>>> &feat_uvs,
                               std::unordered_map<size_t, std::unordered_map<size_t, std::vector<Eigen::VectorXf>>> &feat_uvs_norm,
                               std::unordered_map<size_t, std::unordered_map<size_t, std::vector<double>>> &feat_timestamps) {
            feat_posinG = hist_feat_posinG;
            feat_uvs = hist_feat_uvs;
            feat_uvs_norm = hist_feat_uvs_norm;
            feat_timestamps = hist_feat_timestamps;
        }

        /**
         * @brief Get the boolean of whether to reset the VIO path in the visualization.
         * @return The value of clear_vio_path.
         */
        bool clear_vio_path_in_visualization(){
            return clear_vio_path;
        }

        /**
         * @brief Set the value of clear_vio_path.
         * @param value Set clear_vio_path to value.
         */
        void set_clear_vio_path_in_visualization(bool value){
            if (clear_vio_path != value){
                clear_vio_path = value;
            }
        }


    protected:
        /**
         * @brief Check whether we need to reset the system.
         */
        void check_system_divergence();

        /**
         * @brief This function will try to initialize the state.
         *
         * This should call on our initializer and try to init the state.
         * In the future we should call the structure-from-motion code from here.
         * This function could also be repurposed to re-initialize the system after failure.         *
         * @return True if we have successfully initialized
         */
        bool try_to_initialize();


        /**
         * @brief This will do the propagation and feature updates to the state
         * @param timestamp The most recent timestamp we have tracked to
         */
        void do_feature_propagate_update(double timestamp);


        /**
         * @brief This function will update our historical tracking information.
         * This historical information includes the best estimate of a feature in the global frame.
         * For all features it also has the normalized and raw pixel coordinates at each timestep.
         * The state is also recorded after it is marginalized out of the state.
         * @param features Features using in the last update phase
         */
        void update_keyframe_historical_information(const std::vector<Feature*> &features);


        /// Manager parameters
        VioManagerOptions params;

        /// Our master state object :D
        State* state;

        /// Propagator of our state
        Propagator* propagator;

        /// Our sparse feature tracker (klt or descriptor)
        TrackBase* trackFEATS = nullptr;

        /// Our aruoc tracker
        TrackBase* trackARUCO = nullptr;

        /// State initializer
        InertialInitializer* initializer;

        /// Whether to reset the VIO path for visualization.
        std::atomic<bool> clear_vio_path{false};

        /// Boolean if we are initialized or not
        std::atomic<bool> is_initialized_vio{false};

        /// Our MSCKF feature updater
        UpdaterMSCKF* updaterMSCKF;

        /// Our MSCKF feature updater
        UpdaterSLAM* updaterSLAM;

        /// Our aruoc tracker
        UpdaterZeroVelocity* updaterZUPT = nullptr;

        /// Good features that where used in the last update
        std::vector<Eigen::Vector3d> good_features_MSCKF;

        // Timing statistic file and variables
        std::ofstream of_statistics;
        boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

        // Track how much distance we have traveled
        double timelastupdate = -1;
        double distance = 0;
        /// The inter-frame velocity . (meters/second).
        double velocity = 0;
        /// The consecutive count of the system divergence.
        int consecutive_divergence_count = 0;

        // Startup time of the filter
        double startup_time = -1;

        // If we did a zero velocity update
        bool did_zupt_update = false;
        cv::Mat zupt_image;

        // Historical information of the filter (last marg time, historical states, features seen from all frames)
        double hist_last_marginalized_time = -1;
        std::map<double,Eigen::Matrix<double,7,1>> hist_stateinG;
        std::unordered_map<size_t, Eigen::Vector3d> hist_feat_posinG;
        std::unordered_map<size_t, std::unordered_map<size_t, std::vector<Eigen::VectorXf>>> hist_feat_uvs;
        std::unordered_map<size_t, std::unordered_map<size_t, std::vector<Eigen::VectorXf>>> hist_feat_uvs_norm;
        std::unordered_map<size_t, std::unordered_map<size_t, std::vector<double>>> hist_feat_timestamps;
        /// The ros service reset client.
        // todo (GZ): ideally VIO manager should be ros independent.
        ros::ServiceClient reset_service_client_;

        tbb::task_group tbb_task_group_;
        //tbb::task_scheduler_init task_scheduler_init_(tbb::task_scheduler_init::deferred);

    };


}



#endif //OV_MSCKF_VIOMANAGER_H
