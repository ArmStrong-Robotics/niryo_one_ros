/*
 *  inverse_kinematic.cpp
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
#include <ros/ros.h>

#include "orthopus_space_control/inverse_kinematic.h"

#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <tf/tf.h>

namespace space_control
{
InverseKinematic::InverseKinematic(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number)
  , use_quaternion_(use_quaternion)
  , q_current_(joint_number)
  , q_upper_limit_(joint_number)
  , q_lower_limit_(joint_number)
  , dq_upper_limit_(joint_number)
  , dq_lower_limit_(joint_number)
  , x_current_()
  // , x_min_limit_()
  // , x_max_limit_()
  , sampling_period_(0)
  , qp_init_required_(true)
  , jacobian_init_flag_(true)
{
  ROS_DEBUG_STREAM("InverseKinematic constructor");

  // TODO QUAT improve that !
  space_dimension_ = 7;

  std::vector<double> alpha_weight_vec;
  std::vector<double> beta_weight_vec;
  ros::param::get("~alpha_weight", alpha_weight_vec);
  ros::param::get("~beta_weight", beta_weight_vec);
  ros::param::get("~gamma_weight", gamma_weight_vec);

  setAlphaWeight_(alpha_weight_vec);
  setBetaWeight_(beta_weight_vec);

  ROS_DEBUG("Setting up bounds on joint position (q) of the QP");
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j1/min", q_lower_limit_[0]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j1/max", q_upper_limit_[0]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j2/min", q_lower_limit_[1]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j2/max", q_upper_limit_[1]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j3/min", q_lower_limit_[2]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j3/max", q_upper_limit_[2]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j4/min", q_lower_limit_[3]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j4/max", q_upper_limit_[3]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j5/min", q_lower_limit_[4]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j5/max", q_upper_limit_[4]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j6/min", q_lower_limit_[5]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j6/max", q_upper_limit_[5]);
  /* HACK: Limit joint_3 max reach to prevent a singularity observe in some cases. */
  q_upper_limit_[2] = 1.4;

  /*
   * To limit the joint velocity, we define a constraint for the QP optimisation. Assuming a commun limit of dq_max for
   * all joints, we could write the lower and upper bounds as :
   *        (-dq_max) < dq < dq_max
   * so :
   *      lb = -dq_max
   *      ub = dq_max
   */
  ROS_DEBUG("Setting up bounds on joint velocity (dq) of the QP");
  double dq_max;
  ros::param::get("~joint_max_vel", dq_max);
  JointVelocity limit = JointVelocity(joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    limit[i] = dq_max;
  }
  setDqBounds(limit);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();
  joint_model_group_ = kinematic_model_->getJointModelGroup("arm");

  // TODO Comment that part
  // for (int i = 0; i < 6; i++)
  // {
  //   x_min_limit_[i] = 0.0;
  //   x_max_limit_[i] = 0.0;
  // }
}

void InverseKinematic::init(const std::string end_effector_link, const double sampling_period)
{
  end_effector_link_ = end_effector_link;
  sampling_period_ = sampling_period;
}

void InverseKinematic::setAlphaWeight_(const std::vector<double>& alpha_weight)
{
  // Minimize cartesian velocity (dx) weight
  alpha_weight_ = MatrixXd::Identity(space_dimension_, space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    alpha_weight_(i, i) = alpha_weight[i];
  }
  ROS_DEBUG_STREAM("Set alpha weight to : \n" << alpha_weight_ << "\n");
}

void InverseKinematic::setBetaWeight_(const std::vector<double>& beta_weight)
{
  // Minimize joint velocity (dq) weight
  beta_weight_ = MatrixXd::Identity(joint_number_, joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    beta_weight_(i, i) = beta_weight[i];
  }
  ROS_DEBUG_STREAM("Set alpha weight to : \n" << beta_weight_ << "\n");
}

void InverseKinematic::setGammaWeight_(const std::vector<double>& gamma_weight, const Eigen::Quaterniond& q)
{
  // Minimize cartesian velocity : dx
  gamma_weight_ = MatrixXd::Identity(space_dimension_, space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    gamma_weight_(i, i) = gamma_weight[i];
  }
  double w = q.w(), x = q.x(), y = q.y(), z = q.z();
  Eigen::MatrixXd quat_transfert_matrix(4, 3);
  quat_transfert_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
  Eigen::MatrixXd quat_transfert_matrix_pseudo_inv(3, 4);
  quat_transfert_matrix_pseudo_inv = 4 * quat_transfert_matrix.transpose() / (w * w + x * x + y * y + z * z);
  gamma_weight_.bottomRightCorner(4, 4) = quat_transfert_matrix_pseudo_inv.transpose() *
                                          gamma_weight_.bottomRightCorner(3, 3) * quat_transfert_matrix_pseudo_inv;

  ROS_DEBUG_STREAM("Set alpha weight to : \n" << gamma_weight_ << "\n");
}

void InverseKinematic::setQCurrent(const JointPosition& q_current)
{
  q_current_ = q_current;
}

void InverseKinematic::setOmega(const SpaceVelocity& omega)
{
  dx_omega_ = omega;
}

void InverseKinematic::setXCurrent(const SpacePosition& x_current)
{
  x_current_ = x_current;
  /* As most of computation in this class are eigen matrix operation,
  the current state is also copied in an eigen vector */
  x_current_eigen_ = VectorXd(space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    x_current_eigen_(i) = x_current[i];
  }

  // TODO QUAT improve that to allow copy
  // x_current_eigen_ = x_current.getRawVector();
}

void InverseKinematic::setDqBounds(const JointVelocity& dq_bound)
{
  for (int i = 0; i < dq_bound.size(); i++)
  {
    dq_lower_limit_[i] = -dq_bound[i];
    dq_upper_limit_[i] = dq_bound[i];
  }
}

void InverseKinematic::reset()
{
  /* Initialize a flag used to init QP if required */
  ROS_ERROR("InverseKinematic::reset");
  qp_init_required_ = true;
  jacobian_init_flag_ = true;
  for (int i = 0; i < space_dimension_; i++)
  {
    reset_axis[i] = true;
    reset_axis2[i] = true;
  }
}

void InverseKinematic::resolveInverseKinematic(JointVelocity& dq_computed, const SpaceVelocity& dx_desired)
{
  if (qp_init_required_)
  {
    x_des = x_current_eigen_;  // HACK : store the initial space position
  }
  /* Store desired velocity in eigen vector */
  VectorXd dx_desired_eigen = VectorXd(space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    dx_desired_eigen(i) = dx_desired[i];
  }

  /********* TEST WITH CONSTRAINT *********/
  double epsilon = 0.005;
  double infinite = 1.0;

  if (qp_init_required_)
  {
    for (int i = 0; i < 3; i++)
    {
      x_min_limit[i] = x_current_[i];
      x_max_limit[i] = x_current_[i];
    }
  }
  for (int i = 0; i < 3; i++)
  {
    if (dx_desired[i] != 0.0)
    {
      x_min_limit[i] = x_current_[i] - infinite;
      x_max_limit[i] = x_current_[i] + infinite;
      reset_axis2[i] = true;
    }
    else
    {
      if (reset_axis2[i] == true)
      {
        x_min_limit[i] = x_current_[i] - epsilon;
        x_max_limit[i] = x_current_[i] + epsilon;
        reset_axis2[i] = false;
      }
    }
  }

  /*
    double d_theta_min[7] = {};
    double d_theta_max[7] = {};
    double epsilon_rot = 0.01;
    double inf_rot = 0.1;
    for (int i = 4; i < 7; i++)
    {
      if (dx_omega_[i] != 0.0)
      {
        d_theta_min[i] = -inf_rot;
        d_theta_max[i] = +inf_rot;
      }
      else
      {
        d_theta_min[i] = -epsilon_rot;
        d_theta_max[i] = +epsilon_rot;
      }
    }
    double w = x_current_.getQw(), x = x_current_.getQx(), y = x_current_.getQy(), z = x_current_.getQz();
    Eigen::MatrixXd quat_transfert_matrix(4, 3);
    quat_transfert_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
    Eigen::VectorXd d_theta_eigen(3);
    d_theta_eigen(0) = d_theta_min[4];
    d_theta_eigen(1) = d_theta_min[5];
    d_theta_eigen(2) = d_theta_min[6];
    Eigen::VectorXd d_quat_min = 0.5 * quat_transfert_matrix * d_theta_eigen;
    d_theta_eigen(0) = d_theta_max[4];
    d_theta_eigen(1) = d_theta_max[5];
    d_theta_eigen(2) = d_theta_max[6];
    Eigen::VectorXd d_quat_max = 0.5 * quat_transfert_matrix * d_theta_eigen;

    for (int i = 3; i < 7; i++)
    {
      x_min_limit[i] = x_current_[i] + d_quat_min(i - 3);
      x_max_limit[i] = x_current_[i] + d_quat_max(i - 3);
    }

    ROS_ERROR("x_min_limit = %5f, %5f, %5f, %5f, %5f, %5f, %5f", x_min_limit[0], x_min_limit[1], x_min_limit[2],
              x_min_limit[3], x_min_limit[4], x_min_limit[5], x_min_limit[6]);
    ROS_ERROR("x_max_limit = %5f, %5f, %5f, %5f, %5f, %5f, %5f", x_max_limit[0], x_max_limit[1], x_max_limit[2],
              x_max_limit[3], x_max_limit[4], x_max_limit[5], x_max_limit[6]);
  */
  /********* TEST WITH POS CONTROL *********/
  Eigen::Quaterniond quat_cool(x_current_.getQw(), x_current_.getQx(), x_current_.getQy(), x_current_.getQz());
  setGammaWeight_(gamma_weight_vec, quat_cool);

  for (int i = 0; i < 7; i++)
  {
    if (dx_desired[i] != 0.0)
    {
      reset_axis[i] = true;
    }
    else
    {
      if (reset_axis[i] == true)
      {
        x_des[i] = x_current_[i];
        reset_axis[i] = false;
      }
    }
    // x_des[i] = x_current_[i];
    // compute desired position according to taylor development
    x_des[i] = x_des[i] + dx_desired[i] * sampling_period_;
  }
  ROS_DEBUG_STREAM("x_des = \n" << x_des);
  Eigen::Quaterniond normalize_x_des(x_des[3], x_des[4], x_des[5], x_des[6]);
  normalize_x_des.normalize();
  ROS_WARN_STREAM("diff w = " << x_des[3] - normalize_x_des.w());
  ROS_WARN_STREAM("diff x = " << x_des[4] - normalize_x_des.x());
  ROS_WARN_STREAM("diff y = " << x_des[5] - normalize_x_des.y());
  ROS_WARN_STREAM("diff z = " << x_des[6] - normalize_x_des.z());
  x_des[3] = normalize_x_des.w();
  x_des[4] = normalize_x_des.x();
  x_des[5] = normalize_x_des.y();
  x_des[6] = normalize_x_des.z();
  ROS_DEBUG_STREAM("x_des (normalize) = \n" << x_des);

  ROS_DEBUG_STREAM(
      "|x_des (quat)| = \n"
      << sqrt((x_des[3] * x_des[3]) + (x_des[4] * x_des[4]) + (x_des[5] * x_des[5]) + (x_des[6] * x_des[6])));

  // double norm = sqrt(dx_desired.getQx() * dx_desired.getQx() + dx_desired.getQy() * dx_desired.getQy() +
  //                    dx_desired.getQz() * dx_desired.getQz());
  // ROS_DEBUG_STREAM("norm = \n" << norm);

  // tf::Quaternion delta_q;
  // delta_q.setW(cos(norm * sampling_period_ / 2.0));
  // if (norm != 0.0)
  // {
  //   delta_q.setX(dx_desired.getQx() * sin(norm * sampling_period_ / 2.0) / norm);
  //   delta_q.setY(dx_desired.getQy() * sin(norm * sampling_period_ / 2.0) / norm);
  //   delta_q.setZ(dx_desired.getQz() * sin(norm * sampling_period_ / 2.0) / norm);
  // }
  // else
  // {
  //   delta_q.setX(0.0);
  //   delta_q.setY(0.0);
  //   delta_q.setZ(0.0);
  // }

  // tf::Quaternion init_quat(x_current_.getQx(), x_current_.getQy(), x_current_.getQz(), x_current_.getQw());
  // const tf::Quaternion des_quat = delta_q * init_quat;
  // SpacePosition x_des_quat;
  // x_des_quat.setOrientation(des_quat);

  // ROS_DEBUG_STREAM("des_quat = \n" << des_quat);
  // ROS_DEBUG_STREAM("x_des_quat = \n" << x_des_quat);
  // ROS_DEBUG_STREAM(
  //     "|x_des_quat| = " << sqrt(x_des_quat.getQw() * x_des_quat.getQw() + x_des_quat.getQx() * x_des_quat.getQx() +
  //                               x_des_quat.getQy() * x_des_quat.getQy() + x_des_quat.getQz() *
  //                               x_des_quat.getQz()));

  // x_des[3] = x_des_quat.getQw();
  // x_des[4] = x_des_quat.getQx();
  // x_des[5] = x_des_quat.getQy();
  // x_des[6] = x_des_quat.getQz();

  // updateAxisConstraints_();

  /* Set kinemtic state of the robot to the current joint positions */
  kinematic_state_->setVariablePositions(q_current_);
  kinematic_state_->updateLinkTransforms();
  /* Get jacobian from kinematic state (Moveit) */
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;

  // kinematic_state_->getJacobian(joint_model_group_, kinematic_state_->getLinkModel(end_effector_link_),
  //                               reference_point_position, jacobian, true);
  const robot_model::JointModel* root_joint_model = joint_model_group_->getJointModels()[0];
  getJacobian_(kinematic_state_, joint_model_group_, kinematic_state_->getLinkModel(end_effector_link_),
               root_joint_model->getParentLinkModel(), reference_point_position, jacobian, true);

  /*
   * qpOASES solves QPs of the following form :
   * [1]    min   1/2*x'Hx + x'g
   *        s.t.  lb  <=  x <= ub
   *             lbA <= Ax <= ubA
   * where :
   *      - x' is the transpose of x
   *      - H is the hesiian matrix
   *      - g is the gradient vector
   *      - lb and ub are respectively the lower and upper bound constraint vectors
   *      - lbA and ubA are respectively the lower and upper inequality constraint vectors
   *      - A is the constraint matrix
   *
   * .:!:. NOTE : the symbol ' refers to the transpose of a matrix
   *
   * === Context ==============================================================================================
   * Usually, to resolve inverse kinematic, people use the well-known inverse jacobian formula :
   *        dX = J.dq     =>      dq = J^-1.dX
   * where :
   *      - dX is the cartesian velocity
   *      - dq is the joint velocity
   *      - J the jacobian
   * Unfortunatly, this method lacks when robot is in singular configuration and can lead to unstabilities.
   * ==========================================================================================================
   *
   * In our case, to resolve the inverse kinematic, we use a QP to minimize the joint velocity (dq). This minimization
   * intends to follow a cartesian velocity (dX_des) and position (X_des) trajectory while minimizing joint
   * velocity (dq) :
   *        min(dq) (1/2 * || dX - dX_des ||_alpha^2 + 1/2 * || dq ||_beta^2 + 1/2 * || X - X_des ||_gamma^2)
   * where :
   *      - alpha is the cartesian velocity weight matrix
   *      - beta is the joint velocity weight matrix
   *      - gamma is the cartesian position weight matrix
   *
   * Knowing that dX = J.dq, we have :
   * [2]    min(dq) (1/2 * || J.dq - dX_des ||_alpha^2 + 1/2 * || dq ||_beta^2 + 1/2 * || X - X_des ||_gamma^2)
   *
   * In order to reduce X, we perform a Taylor development (I also show how I develop this part):
   * [3]      1/2 * || X - X_des ||_gamma^2 = 1/2 * || X_0 + T.dX - X_des ||_gamma^2
   *                                      = 1/2 * || X_0 + T.J.dq - X_des ||_gamma^2
   *                                      = 1/2 * (X_0'.gamma.X_0 + dq'.J'.gamma.J.dq*T^2 + X_des'.gamma.X_des)
   *                                      + dq'.J'.gamma.X_0*T - X_des'.gamma.X_0 - X_des'.gamma.J.dq*T
   * where :
   *      - X_0 is the initial position
   *
   * Then, after developing the rest of the equation [2] as shown in [3]:
   *        min(dq) (1/2 * dq'.(J'.alpha.J + beta + J'.gamma.J*T^2).dq
   *               + dq'(-J'.alpha.dX_des + (J'.gamma.X_0 - J'.gamma.Xdes)*T))
   *
   * After identification with [1], we have :
   *        H = J'.alpha.J + beta + J'.gamma.J*T^2
   *        g = -J'.alpha.dX_des + (J'.gamma.X_0 - J'.gamma.Xdes)*T
   *
   */
  ROS_DEBUG("jacobian");
  ROS_DEBUG_STREAM(jacobian);
  /* Hessian computation */
  MatrixXd hessian = (jacobian.transpose() * alpha_weight_ * jacobian) + beta_weight_ +
                     (jacobian.transpose() * sampling_period_ * gamma_weight_ * sampling_period_ * jacobian);

  /* Gradient vector computation */
  VectorXd g = (-jacobian.transpose() * alpha_weight_ * dx_desired_eigen) +
               (jacobian.transpose() * sampling_period_ * gamma_weight_ * x_current_eigen_) -
               (jacobian.transpose() * sampling_period_ * gamma_weight_ * x_des);
  for (int i = 0; i < space_dimension_; i++)
  {
    ROS_ERROR("Delta between x_current and x_des for axis %d : %2f mm", i, (x_current_eigen_[i] - x_des[i]) * 1000.0);
  }
  /* In order to limit the joint position, we define a inequality constraint for the QP optimisation.
   * Taylor developpement of joint position is :
   *        q = q_0 + dq*T
   * with
   *      - q_0 the initial joint position.
   *
   * So affine inequality constraint could be written as :
   *        q_min < q < q_max
   *        q_min < q0 + dq*T < q_max
   *        (q_min-q0) < dq.T < (q_max-q0)
   *
   * so :
   *        A = I.T
   *        lbA = q_min-q0
   *        ubA = q_max-q0
   * where :
   *      - I is the identity matrix
   */
  Eigen::Matrix<double, 12, 6, Eigen::RowMajor> A;
  A.topLeftCorner(6, 6) = Eigen::MatrixXd::Identity(6, 6) * sampling_period_;
  A.block(6, 0, 3, 6) = jacobian.topLeftCorner(3, 6) * sampling_period_;

  /* constraint of quaternion part */
  double w = x_current_.getQw(), x = x_current_.getQx(), y = x_current_.getQy(), z = x_current_.getQz();
  Eigen::MatrixXd quat_transfert_matrix(4, 3);
  quat_transfert_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
  Eigen::MatrixXd quat_transfert_matrix_pseudo_inv(3, 4);
  quat_transfert_matrix_pseudo_inv = 4 * quat_transfert_matrix.transpose() / (w * w + x * x + y * y + z * z);

  A.bottomRightCorner(3, 6) =
      2.0 * quat_transfert_matrix_pseudo_inv * jacobian.bottomRightCorner(4, 6) * sampling_period_;
  // x_min_limit[3] = dx_omega_[4] * sampling_period_;
  // x_min_limit[4] = dx_omega_[5] * sampling_period_;
  // x_min_limit[5] = dx_omega_[6] * sampling_period_;
  // x_max_limit[3] = dx_omega_[4] * sampling_period_;
  // x_max_limit[4] = dx_omega_[5] * sampling_period_;
  // x_max_limit[5] = dx_omega_[6] * sampling_period_;
  x_min_limit[3] = -std::abs(dx_omega_[4]) * sampling_period_;
  x_min_limit[4] = -std::abs(dx_omega_[5]) * sampling_period_;
  x_min_limit[5] = -std::abs(dx_omega_[6]) * sampling_period_;

  x_max_limit[3] = std::abs(dx_omega_[4]) * sampling_period_;
  x_max_limit[4] = std::abs(dx_omega_[5]) * sampling_period_;
  x_max_limit[5] = std::abs(dx_omega_[6]) * sampling_period_;

  ROS_ERROR("x_min_limit = %5f, %5f, %5f, %5f, %5f, %5f, %5f", x_min_limit[0], x_min_limit[1], x_min_limit[2],
            x_min_limit[3], x_min_limit[4], x_min_limit[5], x_min_limit[6]);
  ROS_ERROR("x_max_limit = %5f, %5f, %5f, %5f, %5f, %5f, %5f", x_max_limit[0], x_max_limit[1], x_max_limit[2],
            x_max_limit[3], x_max_limit[4], x_max_limit[5], x_max_limit[6]);

  double lbA[] = { /* Joints min hard limits constraints */
                   (q_lower_limit_[0] - q_current_[0]),
                   (q_lower_limit_[1] - q_current_[1]),
                   (q_lower_limit_[2] - q_current_[2]),
                   (q_lower_limit_[3] - q_current_[3]),
                   (q_lower_limit_[4] - q_current_[4]),
                   (q_lower_limit_[5] - q_current_[5]),
                   (x_min_limit[0] - x_current_eigen_(0, 0)),
                   (x_min_limit[1] - x_current_eigen_(1, 0)),
                   (x_min_limit[2] - x_current_eigen_(2, 0)),
                   (x_min_limit[3] - x_current_eigen_(3, 0)),
                   (x_min_limit[4] - x_current_eigen_(4, 0)),
                   (x_min_limit[5] - x_current_eigen_(5, 0)),
                   (x_min_limit[6] - x_current_eigen_(6, 0)),
                   x_min_limit[3],
                   x_min_limit[4],
                   x_min_limit[5]

  };

  double ubA[] = { /* Joints max hard limits constraints */
                   (q_upper_limit_[0] - q_current_[0]),
                   (q_upper_limit_[1] - q_current_[1]),
                   (q_upper_limit_[2] - q_current_[2]),
                   (q_upper_limit_[3] - q_current_[3]),
                   (q_upper_limit_[4] - q_current_[4]),
                   (q_upper_limit_[5] - q_current_[5]),
                   (x_max_limit[0] - x_current_eigen_(0, 0)),
                   (x_max_limit[1] - x_current_eigen_(1, 0)),
                   (x_max_limit[2] - x_current_eigen_(2, 0)),
                   x_max_limit[3],
                   x_max_limit[4],
                   x_max_limit[5]
  };

  /* Solve QP */
  qpOASES::real_t xOpt[6];
  qpOASES::int_t nWSR = 10;
  int qp_return = 0;
  if (qp_init_required_)
  {
    /* Initialize QP solver */
    QP_ = new qpOASES::SQProblem(6, 6);  // HACK : ONLY JOINT VELOCITY LIMIT
    qpOASES::Options options;
    options.setToReliable();
    // options.printLevel = qpOASES::PL_NONE;
    QP_->setOptions(options);
    qp_return = QP_->init(hessian.data(), g.data(), A.data(), dq_lower_limit_.data(), dq_upper_limit_.data(), lbA, ubA,
                          nWSR, 0);
    qp_init_required_ = false;
  }
  else
  {
    qp_return = QP_->hotstart(hessian.data(), g.data(), A.data(), dq_lower_limit_.data(), dq_upper_limit_.data(), lbA,
                              ubA, nWSR, 0);
  }

  if (qp_return == qpOASES::SUCCESSFUL_RETURN)
  {
    ROS_DEBUG_STREAM("qpOASES : succesfully return");

    /* Get solution of the QP */
    QP_->getPrimalSolution(xOpt);
    dq_computed[0] = xOpt[0];
    dq_computed[1] = xOpt[1];
    dq_computed[2] = xOpt[2];
    dq_computed[3] = xOpt[3];
    dq_computed[4] = xOpt[4];
    dq_computed[5] = xOpt[5];
  }
  else
  {
    ROS_ERROR("qpOASES : Failed with code : %d !", qp_return);
    dq_computed[0] = 0.0;
    dq_computed[1] = 0.0;
    dq_computed[2] = 0.0;
    dq_computed[3] = 0.0;
    dq_computed[4] = 0.0;
    dq_computed[5] = 0.0;
  }

  if (qp_return != qpOASES::SUCCESSFUL_RETURN && qp_return != qpOASES::RET_MAX_NWSR_REACHED)
  {
    exit(0);  // TODO improve error handling. Crash of the application is neither safe nor beautiful
  }
}

void InverseKinematic::updateAxisConstraints_()
{
  for (int i = 0; i < 6; i++)
  {
    if (request_update_constraint_[i])
    {
      ROS_WARN_STREAM("Update axis " << i << " constraints with new tolerance of "
                                     << request_update_constraint_tolerance_[i] << " m.");

      // x_min_limit_[i] = x_current_eigen_(i, 0) - request_update_constraint_tolerance_[i];
      // x_max_limit_[i] = x_current_eigen_(i, 0) + request_update_constraint_tolerance_[i];

      request_update_constraint_[i] = false;

      /*
       * Update the current desired cartesian position.
       * This allows the axis to drift and so to be control.
       * It is a dirty way of handle that !
       */
      // x_des(i, 0) = x_current_eigen_(i, 0);
      ROS_WARN_STREAM("Update the new desired position of " << i << " to " << x_des(i, 0));
    }
  }
}

void InverseKinematic::requestUpdateAxisConstraints(int axis, double tolerance)
{
  ROS_WARN_STREAM("Update axis " << axis << " constraints with new tolerance of " << tolerance << "");
  request_update_constraint_[axis] = true;
  request_update_constraint_tolerance_[axis] = tolerance;
}

bool InverseKinematic::getJacobian_(const robot_state::RobotStatePtr kinematic_state,
                                    const robot_state::JointModelGroup* group, const robot_state::LinkModel* link,
                                    const robot_model::LinkModel* root_link_model,
                                    const Eigen::Vector3d& reference_point_position, Eigen::MatrixXd& jacobian,
                                    bool use_quaternion_representation)
{
  if (!group->isChain())
  {
    ROS_ERROR("The group '%s' is not a chain. Cannot compute Jacobian.", group->getName().c_str());
    return false;
  }

  if (!group->isLinkUpdated(link->getName()))
  {
    ROS_ERROR("Link name '%s' does not exist in the chain '%s' or is not a child for this chain",
              link->getName().c_str(), group->getName().c_str());
    return false;
  }
  const robot_model::JointModel* root_joint_model = group->getJointModels()[0];
  Eigen::Affine3d reference_transform = root_link_model ?
                                            kinematic_state->getGlobalLinkTransform(root_link_model).inverse() :
                                            Eigen::Affine3d::Identity();
  int rows = use_quaternion_representation ? 7 : 6;
  int columns = group->getVariableCount();
  jacobian = Eigen::MatrixXd::Zero(rows, columns);

  Eigen::Affine3d link_transform = reference_transform * kinematic_state->getGlobalLinkTransform(link);
  Eigen::Vector3d point_transform = link_transform * reference_point_position;

  Eigen::Vector3d joint_axis;
  Eigen::Affine3d joint_transform;

  while (link)
  {
    const robot_model::JointModel* pjm = link->getParentJointModel();
    if (pjm->getVariableCount() > 0)
    {
      unsigned int joint_index = group->getVariableGroupIndex(pjm->getName());
      if (pjm->getType() == robot_model::JointModel::REVOLUTE)
      {
        joint_transform = reference_transform * kinematic_state->getGlobalLinkTransform(link);
        joint_axis = joint_transform.rotation() * static_cast<const robot_model::RevoluteJointModel*>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) =
            jacobian.block<3, 1>(0, joint_index) + joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index) = jacobian.block<3, 1>(3, joint_index) + joint_axis;
      }
      else if (pjm->getType() == robot_model::JointModel::PRISMATIC)
      {
        joint_transform = reference_transform * kinematic_state->getGlobalLinkTransform(link);
        joint_axis = joint_transform * static_cast<const robot_model::PrismaticJointModel*>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
      }
      else if (pjm->getType() == robot_model::JointModel::PLANAR)
      {
        joint_transform = reference_transform * kinematic_state->getGlobalLinkTransform(link);
        joint_axis = joint_transform * Eigen::Vector3d(1.0, 0.0, 0.0);
        jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
        joint_axis = joint_transform * Eigen::Vector3d(0.0, 1.0, 0.0);
        jacobian.block<3, 1>(0, joint_index + 1) = jacobian.block<3, 1>(0, joint_index + 1) + joint_axis;
        joint_axis = joint_transform * Eigen::Vector3d(0.0, 0.0, 1.0);
        jacobian.block<3, 1>(0, joint_index + 2) = jacobian.block<3, 1>(0, joint_index + 2) +
                                                   joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index + 2) = jacobian.block<3, 1>(3, joint_index + 2) + joint_axis;
      }
      else
        ROS_ERROR("Unknown type of joint in Jacobian computation");
    }
    if (pjm == root_joint_model)
      break;
    link = pjm->getParentLinkModel();
  }
  if (use_quaternion_representation)
  {  // Quaternion representation
    // From "Advanced Dynamics and Motion Simulation" by Paul Mitiguy
    // d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [ omega_1 ]
    //        [x]           [  w -z  y ]    [ omega_2 ]
    //        [y]           [  z  w -x ]    [ omega_3 ]
    //        [z]           [ -y  x  w ]
    Eigen::Quaterniond q(link_transform.rotation());
    Eigen::Quaterniond* final_quat;
    if (jacobian_init_flag_)
    {
      jacobian_init_flag_ = false;
      final_quat = new Eigen::Quaterniond(q);
    }
    else
    {
      double diff_norm = sqrt(pow(q.w() - jacobian_q_prev_.w(), 2) + pow(q.x() - jacobian_q_prev_.x(), 2) +
                              pow(q.y() - jacobian_q_prev_.y(), 2) + pow(q.z() - jacobian_q_prev_.z(), 2));

      ROS_WARN("diff_norm (jacobian) = %5f", diff_norm);
      if (diff_norm > 1)
      {
        final_quat = new Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
      }
      else
      {
        final_quat = new Eigen::Quaterniond(q);
      }
    }
    jacobian_q_prev_ = *final_quat;

    double w = final_quat->w(), x = final_quat->x(), y = final_quat->y(), z = final_quat->z();
    Eigen::MatrixXd quaternion_update_matrix(4, 3);
    quaternion_update_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
    jacobian.block(3, 0, 4, columns) = 0.5 * quaternion_update_matrix * jacobian.block(3, 0, 3, columns);
  }
  return true;
}
}