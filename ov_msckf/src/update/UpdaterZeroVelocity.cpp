/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
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
#include "UpdaterZeroVelocity.h"
#include <ros/console.h>


using namespace ov_msckf;



bool UpdaterZeroVelocity::try_update(State *state, double timestamp) {

    // Return if we don't have any imu data yet
    if(imu_data.empty())
        return false;

    // Set the last time offset value if we have just started the system up
    if(!have_last_prop_time_offset) {
        last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0);
        have_last_prop_time_offset = true;
    }

    // assert that the time we are requesting is in the future
    assert(timestamp > state->_timestamp);

    // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
    double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);

    // First lets construct an IMU vector of measurements we need
    //double time0 = state->_timestamp+t_off_new;
    //double time0 = state->_timestamp+last_prop_time_offset;
    //double time1 = timestamp+t_off_new;

    // Select bounding inertial measurements
    // TODO(guoxiang): if there is a missing image in the stream, we will have a problem. Since the difference between time0 and time1 is too big.
    // std::vector<Propagator::IMUDATA> imu_recent = Propagator::select_imu_readings(imu_data, time0, time1);
    std::vector<Propagator::IMUDATA> imu_recent;
    try {
        imu_recent = Propagator::select_imu_readings(imu_data, timestamp - _imu_zero_velocity_window_length, timestamp);
    }
    catch (...) {
        ROS_WARN_STREAM("Cannot fetch required IMU measurements from stored IMU data.");
        return false;
    }

    // Move forward in time
    last_prop_time_offset = t_off_new;

    // Check that we have at least one measurement to propagate with
    if(imu_recent.empty() || imu_recent.size() < 2) {
        ROS_WARN_THROTTLE(3, "There are not enough IMU data to check for zero velocity.");
        return false;
    }

    ROS_DEBUG_THROTTLE(3, "The number of selected IMU measurements for zero velocity update is: %zu (Debugging is throttled: 3s)", imu_recent.size());

    Eigen::Vector3d linsum = Eigen::Vector3d::Zero();
    for(size_t i=0; i < imu_recent.size(); i++) {
      linsum += imu_recent.at(i).am;
    }
    Eigen::Vector3d linavg = linsum/imu_recent.size();

    // Check IMU acceleration variance.
    double a_var = 0;
    for(auto data : imu_recent) {
      a_var += (data.am-linavg).dot(data.am-linavg);
    }

    a_var = std::sqrt(a_var/(imu_recent.size()-1));

    ROS_DEBUG_THROTTLE(3, "IMU acceleration variance value in the zero velocity detection window: %.4f / %.4f. (Debugging is throttled: 3s)", a_var, _imu_zero_velocity_threshold);
    // If acceleration variance is above the threshold, this means the agent is not standstill. Return false.
    if(a_var >= _imu_zero_velocity_threshold) {
      return false;
    }

    // todo: GZ, this is simple zero velocity logic, need further check. Need to improve!!
    /*
    // If we should integrate the acceleration and say the velocity should be zero
    // Also if we should still inflate the bias based on their random walk noises
    bool integrated_accel_constraint = false;
    bool model_time_varying_bias = true;

    // Order of our Jacobian
    std::vector<Type*> Hx_order;
    Hx_order.push_back(state->_imu->q());
    Hx_order.push_back(state->_imu->bg());
    Hx_order.push_back(state->_imu->ba());
    if(integrated_accel_constraint) Hx_order.push_back(state->_imu->v());

    // Large final matrices used for update
    int h_size = (integrated_accel_constraint) ? 12 : 9;
    int m_size = 6*(imu_recent.size()-1);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size,h_size);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m_size,m_size);

    // Loop through all our IMU and construct the residual and Jacobian
    // State order is: [q_GtoI, bg, ba, v_IinG]
    // Measurement order is: [w_true = 0, a_true = 0 or v_k+1 = 0]
    // w_true = w_m - bw - nw
    // a_true = a_m - ba - R*g - na
    // v_true = v_k - g*dt + R^T*(a_m - ba - na)*dt
    double dt_summed = 0;
    for(size_t i=0; i<imu_recent.size()-1; i++) {

        // Precomputed values
        double dt = imu_recent.at(i+1).timestamp - imu_recent.at(i).timestamp;
        Eigen::Vector3d a_hat = imu_recent.at(i).am - state->_imu->bias_a();

        // Measurement residual (true value is zero)
        res.block(6*i+0,0,3,1) = -(imu_recent.at(i).wm - state->_imu->bias_g());
        if(!integrated_accel_constraint) {
            res.block(6*i+3,0,3,1) = -(a_hat - state->_imu->Rot()*_gravity);
        } else {
            res.block(6*i+3,0,3,1) = -(state->_imu->vel() - _gravity*dt + state->_imu->Rot().transpose()*a_hat*dt);
        }

        // Measurement Jacobian
        Eigen::Matrix3d R_GtoI_jacob = (state->_options.do_fej)? state->_imu->Rot_fej() : state->_imu->Rot();
        H.block(6*i+0,3,3,3) = -Eigen::Matrix<double,3,3>::Identity();
        if(!integrated_accel_constraint) {
            H.block(6*i+3,0,3,3) = -skew_x(R_GtoI_jacob*_gravity);
            H.block(6*i+3,6,3,3) = -Eigen::Matrix<double,3,3>::Identity();
        } else {
            H.block(6*i+3,0,3,3) = -R_GtoI_jacob.transpose()*skew_x(a_hat)*dt;
            H.block(6*i+3,6,3,3) = -R_GtoI_jacob.transpose()*dt;
            H.block(6*i+3,9,3,3) = Eigen::Matrix<double,3,3>::Identity();
        }

        // Measurement noise (convert from continuous to discrete)
        // Note the dt time might be different if we have "cut" any imu measurements
        R.block(6*i+0,6*i+0,3,3) *= _noises.sigma_w_2/dt;
        if(!integrated_accel_constraint) {
            R.block(6*i+3,6*i+3,3,3) *= _noises.sigma_a_2/dt;
        } else {
            R.block(6*i+3,6*i+3,3,3) *= _noises.sigma_a_2*dt;
        }
        dt_summed += dt;

    }

    // Multiply our noise matrix by a fixed amount
    // We typically need to treat the IMU as being "worst" to detect / not become over confident
    R *= _zupt_noise_multiplier;

    // Next propagate the biases forward in time
    // NOTE: G*Qd*G^t = dt*Qd*dt = dt*Qc
    Eigen::MatrixXd Q_bias = Eigen::MatrixXd::Identity(6,6);
    Q_bias.block(0,0,3,3) *= dt_summed*_noises.sigma_wb;
    Q_bias.block(3,3,3,3) *= dt_summed*_noises.sigma_ab;

    // Chi2 distance check
    // NOTE: we also append the propagation we "would do before the update" if this was to be accepted
    // NOTE: we don't propagate first since if we fail the chi2 then we just want to return and do normal logic
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    P_marg.block(3,3,6,6) += Q_bias;
    Eigen::MatrixXd S = H*P_marg*H.transpose() + R;
    double chi2 = res.dot(S.llt().solve(res));

    // Get our threshold (we precompute up to 1000 but handle the case that it is more)
    double chi2_check;
    if(res.rows() < 1000) {
        chi2_check = chi_squared_table[res.rows()];
    } else {
        boost::math::chi_squared chi_squared_dist(res.rows());
        chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
        printf(YELLOW "[ZUPT]: chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
    }

    // Check if we are currently zero velocity
    // We need to pass the chi2 and not be above our velocity threshold
    if(chi2 > _options.chi2_multipler*chi2_check || state->_imu->vel().norm() > _zupt_max_velocity) {
        printf(YELLOW "[ZUPT]: rejected zero velocity |v_IinG| = %.3f (chi2 %.3f > %.3f)\n" RESET,state->_imu->vel().norm(),chi2,_options.chi2_multipler*chi2_check);
        return false;
    }

    // Next propagate the biases forward in time
    // NOTE: G*Qd*G^t = dt*Qd*dt = dt*Qc
    if(model_time_varying_bias) {
        Eigen::MatrixXd Phi_bias = Eigen::MatrixXd::Identity(6,6);
        std::vector<Type*> Phi_order;
        Phi_order.push_back(state->_imu->bg());
        Phi_order.push_back(state->_imu->ba());
        StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_bias, Q_bias);
    }

    // Else we are good, update the system
    printf(CYAN "[ZUPT]: accepted zero velocity |v_IinG| = %.3f (chi2 %.3f < %.3f)\n" RESET,state->_imu->vel().norm(),chi2,_options.chi2_multipler*chi2_check);
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
     */

    // Finally move the state time forward
    state->_timestamp = timestamp;
    return true;


}