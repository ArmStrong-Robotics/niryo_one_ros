/*
  TODO copyright
*/

#ifndef CARTESIAN_CONTROLLER_INVERSE_KINEMATIC_H
#define CARTESIAN_CONTROLLER_INVERSE_KINEMATIC_H

#include <ros/ros.h>

// #include "geometry_msgs/TwistStamped.h"
// #include "sensor_msgs/Joy.h"
// #include "std_msgs/Bool.h"

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// QPOASES
#include <qpOASES.hpp>

// Eigen3
#include <Eigen/Dense>

// Messages
#include <geometry_msgs/TwistStamped.h>

typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

namespace cartesian_controller
{
class InverseKinematic
{
public:
  InverseKinematic();
  void Reset(sensor_msgs::JointState& current_joint_state);

  void ResolveInverseKinematic(double (&joint_position_command)[6], sensor_msgs::JointState& current_joint_state,
                               double (&cartesian_velocity_desired)[6]);  // TODO should not use magic number

  void UpdateAxisConstraints(bool (&axis_constraint)[6], double tolerance);
  void setCartesianMode(int mode);
  int getCartesianMode();

protected:
private:
  ros::NodeHandle n_;

  double alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6;
  double beta_1, beta_2, beta_3, beta_4, beta_5, beta_6;
  double gamma_1, gamma_2, gamma_3, gamma_4, gamma_5, gamma_6;
  double joints_limits_max[6];
  double joints_limits_min[6];
  // Weight for cartesian velocity minimization
  Matrix6d alpha_weight;
  // Weight for joint velocity minimization
  Matrix6d beta_weight;
  // Weight for cartesian position minimization
  Matrix6d gamma_weight;
  // QP solver
  qpOASES::SQProblem* IK;

  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;

  bool start_flag = true;
  double theta[6];

  robot_state::JointModelGroup* joint_model_group;
  double x_min;
  double x_max;
  double y_min;
  double y_max;
  double z_min;
  double z_max;
  double r_min;
  double r_max;
  double p_min;
  double p_max;
  double yaw_min;
  double yaw_max;

  double x_des[6];
  double x_min_limit[6];
  double x_max_limit[6];

  Vector6d currentPosition;
  Vector6d desiredPosition;

  int cartesian_mode_;
};
}
#endif
