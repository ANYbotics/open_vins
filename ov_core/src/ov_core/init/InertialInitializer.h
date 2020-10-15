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
#ifndef OV_CORE_INERTIALINITIALIZER_H
#define OV_CORE_INERTIALINITIALIZER_H

#include <Eigen/Eigen>
#include <mutex>
#include <utility>
#include "../utils/quat_ops.h"
#include "../utils/colors.h"

namespace ov_core {



    /**
     * @brief Initializer for visual-inertial system.
     *
     * This class has a series of functions that can be used to initialize your system.
     * Right now we have our implementation that assumes that the imu starts from standing still.
     * In the future we plan to add support for structure-from-motion dynamic initialization.
     *
     * To initialize from standstill:
     * 1. Collect all inertial measurements
     * 2. See if within the last window there was a jump in acceleration
     * 3. If the jump is past our threshold we should init (i.e. we have started moving)
     * 4. Use the *previous* window, which should have been stationary to initialize orientation
     * 5. Return a roll and pitch aligned with gravity and biases.
     *
     */
    class InertialInitializer {

    public:

        /**
         * @brief Struct for a single imu measurement (time, wm, am)
         */
        struct IMUDATA {

            /// Timestamp of the reading
            double timestamp;

            /// Gyroscope reading, angular velocity (rad/s)
            Eigen::Matrix<double, 3, 1> wm;

            /// Accelerometer reading, linear acceleration (m/s^2)
            Eigen::Matrix<double, 3, 1> am;

        };

        /**
         * @brief Struct for a single feet contact measurement (time, is_in_contact)
         */
         struct FeetContact{

           /// Timestamp of the reading
           double timestamp;

           /// Whether the agent is in contact
           bool is_in_contact;
         };


        /**
         * @brief Default constructor
         * @param gravity Gravity in the global frame of reference
         * @param initialization_window_length Amount of time we will use to initialize over (seconds)
         * @param imu_init_threshold Variance threshold on our acceleration to use IMU measurements for initialization
         * @param imu_init_num_of_imu_measurements The number of IMU measurements used for initialization
         */
        InertialInitializer(Eigen::Matrix<double,3,1> gravity, double initialization_window_length, double imu_init_threshold, int imu_init_num_of_imu_measurements) :
                            _gravity(std::move(gravity)),
               _initialization_window_length(initialization_window_length), _imu_init_threshold(imu_init_threshold),
         _imu_init_num_of_imu_measurements(imu_init_num_of_imu_measurements){}


        /**
         * @brief Stores incoming inertial readings
         *
         * @param timestamp Timestamp of imu reading
         * @param wm Gyro angular velocity reading
         * @param am Accelerometer linear acceleration reading
         */
        void feed_imu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am);


        /**
         * @brief Store the incoming feet contact measurements in a container.
         * @param timestamp The timestamp of contact measurements
         * @param is_in_contact The state of the contact. True means it is in contact, false otherwise
         */
        void feed_contact(double timestamp, bool is_in_contact);
        /**
         * @brief Get the window starting time and ending time to select IMU measurements for initialization
         * @param window_start The obtained window starting time
         * @param window_end The obtained window ending time
         * @param consider_contact Whether to obtain window starting time and ending time from the period when the agent is in contact
         * @param initialization_window_length The time duration of the initialization window
         * @return
         */
        bool obtain_initialization_window(double & window_start, double & window_end, bool consider_contact, const double initialization_window_length);


        /**
         * @brief Fetch the valid IMU measurements for initialization
         * @param valid_imu_measurements_window The vector to store the fetched valid IMU measurements
         * @param initialization_window_length The initialization time length (Unit: second)
         * @param num_valid_imu_measurements_threshold The threshold number of the valid IMU measurements needed for initialization
         * @param consider_contact Whether to consider contact when fetching valid IMU measurements needed for initialization
         * @return True if fetching enough valid IMU measurements
         */
        bool fetch_imu_data_for_initialization(std::vector<IMUDATA>& valid_imu_measurements_window, double initialization_window_length, int num_valid_imu_measurements_threshold, bool consider_contact);


        /**
         * @brief Try to initialize the system using just the imu
         *
         * This will check if we have had a large enough jump in our acceleration.
         * If we have then we will use the period of time before this jump to initialize the state.
         * This assumes that our imu is sitting still and is not moving (so this would fail if we are experiencing constant acceleration).
         *
         * In the case that we do not wait for a jump (i.e. `wait_for_jerk` is false), then the system will try to initialize as soon as possible.
         * This is only recommended if you have zero velocity update enabled to handle the stationary cases.
         * To initialize in this case, we need to have the average angular variance be below the set threshold (i.e. we need to be stationary).
         *
         * @param[out] time0 Timestamp that the returned state is at
         * @param[out] q_GtoI0 Orientation at initialization
         * @param[out] b_w0 Gyro bias at initialization
         * @param[out] v_I0inG Velocity at initialization
         * @param[out] b_a0 Acceleration bias at initialization
         * @param[out] p_I0inG Position at initialization
         * @param consider_contact True to start initialization only when the agent is in contact with the ground
         * @return True if we have successfully initialized our system
         */
        bool initialize_with_imu(double &time0, Eigen::Matrix<double,4,1> &q_GtoI0, Eigen::Matrix<double,3,1> &b_w0,
                                 Eigen::Matrix<double,3,1> &v_I0inG, Eigen::Matrix<double,3,1> &b_a0, Eigen::Matrix<double,3,1> &p_I0inG, bool consider_contact=true);


    protected:

        /// Gravity vector
        Eigen::Matrix<double,3,1> _gravity;

        /// Amount of time we will initialize over (seconds)
        double _initialization_window_length;

        /// Variance threshold on our acceleration to use IMU measurements for initialization
        double _imu_init_threshold;

        /// The number of IMU measurements used for initialization
        int _imu_init_num_of_imu_measurements;


        /// Our history of IMU messages (time, angular, linear)
        std::vector<IMUDATA> imu_data;
        mutable std::mutex imu_data_mutex_;

        /// Our history of feet contact messages (time, is_in_contact)
        std::vector<FeetContact> contact_data;
        mutable std::mutex contact_data_mutex_;


    };


}

#endif //OV_CORE_INERTIALINITIALIZER_H
