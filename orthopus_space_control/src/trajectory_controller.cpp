/*
 *  trajectory_controller.cpp
 *  Copyright (C) 2019 Orthopus
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ros/ros.h"

#include "orthopus_space_control/trajectory_controller.h"

namespace space_control
{
TrajectoryController::TrajectoryController(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number), use_quaternion_(use_quaternion), x_traj_desired_(), x_current_(), sampling_period_(0.0)
{
  ROS_DEBUG_STREAM("TrajectoryController constructor");

  // TODO Improvement: put following value in param
  traj_position_tolerance_ = 0.005;    // 5 mm
  traj_orientation_tolerance_ = 0.02;  // 0.01 rad = 0.573 deg // TODO it is not radian in case of quaternion
}

void TrajectoryController::init(double sampling_period)
{
  sampling_period_ = sampling_period;
  is_completed_ = true;
  double p_gain, i_gain, space_position_max_vel;
  ros::param::get("~trajectory_ctrl_p_gain", p_gain);
  ros::param::get("~trajectory_ctrl_i_gain", i_gain);
  ros::param::get("~space_position_max_vel", space_position_max_vel);

  for (int i = 0; i < 7; i++)
  {
    pi_ctrl_[i].init(sampling_period_, -space_position_max_vel, space_position_max_vel);
    pi_ctrl_[i].setGains(p_gain, i_gain);
    euler_factor_[i] = 1.0;
  }
}

void TrajectoryController::reset()
{
  for (int i = 0; i < 7; i++)
  {
    pi_ctrl_[i].reset();
    euler_factor_[i] = 1.0;
  }
  is_completed_ = true;
}

void TrajectoryController::setXCurrent(const SpacePosition& x_current)
{
  x_current_ = x_current;
  euler_factor_[0] = 1.0;
  euler_factor_[1] = 1.0;
  euler_factor_[2] = 1.0;
  euler_factor_[3] = 1.0;
  euler_factor_[4] = 1.0;
  euler_factor_[5] = 1.0;
}

void TrajectoryController::setTrajectoryPose(const SpacePosition& x_pose)
{
  is_completed_ = false;
  x_traj_desired_ = x_pose;
}

void TrajectoryController::computeTrajectory(SpaceVelocity& dx_output)
{
  eulerFlipHandling_();
  updateTrajectoryCompletion_();
  processPi_(dx_output);
}

bool TrajectoryController::isTrajectoryCompleted()
{
  return is_completed_;
}

void TrajectoryController::updateTrajectoryCompletion_()
{
  if (!is_completed_)
  {
    is_completed_ = true;
    for (int i = 0; i < 3; i++)
    {
      if (std::abs(x_traj_desired_[i] - x_current_[i]) > traj_position_tolerance_)
      {
        is_completed_ = false;
        ROS_DEBUG("Trajectory status : delta of %5f on %d axis", x_traj_desired_[i] - x_current_[i], i);
      }
    }
    for (int i = 3; i < 7; i++)
    {
      if (std::abs(x_traj_desired_[i] - x_current_[i]) > traj_orientation_tolerance_)
      {
        is_completed_ = false;
        ROS_DEBUG("Trajectory status : delta of %5f on %d axis", x_traj_desired_[i] - x_current_[i], i);
      }
    }
  }
}

void TrajectoryController::eulerFlipHandling_()
{
}

void TrajectoryController::processPi_(SpaceVelocity& dx_output)
{
  for (int i = 0; i < 7; i++)
  {
    if (i == 3)
    {
      dx_output[i] = 0.0;
    }
    else
    {
      double error = (x_traj_desired_[i] - x_current_[i] * euler_factor_[i]);
      double pi_result;
      ROS_DEBUG("error[%d] : %5f", i, error);
      pi_ctrl_[i].execute(error, pi_result);
      dx_output[i] = pi_result;
    }
  }
}
}
