// TODO copyright
#include <ros/ros.h>

#include "orthopus_interface/inverse_kinematic.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
static const int RATE = 10;  // TODO should be set during class init
static const double CALC_PERIOD = 1.0 / RATE;

namespace cartesian_controller
{
InverseKinematic::InverseKinematic(ros::Publisher &debug_pub_, ros::Publisher &debug_des_pub_)
{
  ros::param::get("~alpha_1", alpha_1);
  ros::param::get("~alpha_2", alpha_2);
  ros::param::get("~alpha_3", alpha_3);
  ros::param::get("~alpha_4", alpha_4);
  ros::param::get("~alpha_5", alpha_5);
  ros::param::get("~alpha_6", alpha_6);
  ros::param::get("~alpha_7", alpha_7);
  ros::param::get("~beta_1", beta_1);
  ros::param::get("~beta_2", beta_2);
  ros::param::get("~beta_3", beta_3);
  ros::param::get("~beta_4", beta_4);
  ros::param::get("~beta_5", beta_5);
  ros::param::get("~beta_6", beta_6);
  ros::param::get("~gamma_1", gamma_1);
  ros::param::get("~gamma_2", gamma_2);
  ros::param::get("~gamma_3", gamma_3);
  ros::param::get("~gamma_4", gamma_4);
  ros::param::get("~gamma_5", gamma_5);
  ros::param::get("~gamma_6", gamma_6);
  ros::param::get("~gamma_7", gamma_7);

  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j1/min", joints_limits_min[0]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j1/max", joints_limits_max[0]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j2/min", joints_limits_min[1]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j2/max", joints_limits_max[1]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j3/min", joints_limits_min[2]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j3/max", joints_limits_max[2]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j4/min", joints_limits_min[3]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j4/max", joints_limits_max[3]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j5/min", joints_limits_min[4]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j5/max", joints_limits_max[4]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j6/min", joints_limits_min[5]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j6/max", joints_limits_max[5]);

  debug_pos_pub_ = debug_pub_;
  debug_pos_des_pub_ = debug_des_pub_;
  
  x_des[0] = 0.2379;
  x_des[1] = 0.0;
  x_des[2] = 0.4175;
  x_des[3] = 0.5;
  x_des[4] = 0.5;
  x_des[5] = 0.5;

  // TODO check init all members
  // TODO for loop
  x_min_limit[0] = 0.0;
  x_min_limit[1] = 0.0;
  x_min_limit[2] = 0.0;
  x_min_limit[3] = 0.0;
  x_min_limit[4] = 0.0;
  x_min_limit[5] = 0.0;

  x_max_limit[0] = 0.0;
  x_max_limit[1] = 0.0;
  x_max_limit[2] = 0.0;
  x_max_limit[3] = 0.0;
  x_max_limit[4] = 0.0;
  x_max_limit[5] = 0.0;

  // Wait for initial messages
  ROS_INFO("Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_INFO("Received first joint msg.");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  ROS_DEBUG("Model frame: %s", kinematic_model->getModelFrame().c_str());
  kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state->setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup("arm");

  // Minimize cartesian velocity : dx
  alpha_weight = Matrix6d::Identity(6, 6);
  alpha_weight(0, 0) = alpha_1;
  alpha_weight(1, 1) = alpha_2;
  alpha_weight(2, 2) = alpha_3;
  alpha_weight(3, 3) = alpha_4;
  alpha_weight(4, 4) = alpha_5;
  alpha_weight(5, 5) = alpha_6;
  ROS_DEBUG_STREAM("alpha_weight: \n" << alpha_weight << "\n");

  // Minimize joints velocities : dq
  beta_weight = Matrix6d::Identity(6, 6);
  beta_weight(0, 0) = beta_1;
  beta_weight(1, 1) = beta_2;
  beta_weight(2, 2) = beta_3;
  beta_weight(3, 3) = beta_4;
  beta_weight(4, 4) = beta_5;
  beta_weight(5, 5) = beta_6;
  ROS_DEBUG_STREAM("beta_weight: \n" << beta_weight << "\n");

  // Minimize cartesian position : x
  gamma_weight = Matrix6d::Identity(6, 6);
  gamma_weight(0, 0) = gamma_1;
  gamma_weight(1, 1) = gamma_2;
  gamma_weight(2, 2) = gamma_3;
  gamma_weight(3, 3) = gamma_4;
  gamma_weight(4, 4) = gamma_5;
  gamma_weight(5, 5) = gamma_6;
  ROS_DEBUG_STREAM("gamma_weight: \n" << gamma_weight << "\n");

  // Initialize QP solver
  IK = new qpOASES::SQProblem(6, 12);
  qpOASES::Options options;
  //options.printLevel = qpOASES::PL_NONE;
  IK->setOptions(options);
}

void InverseKinematic::Init(sensor_msgs::JointState& current_joint_state)
{
  sensor_msgs::JointState local_joint_state = current_joint_state;
  kinematic_state->setVariableValues(local_joint_state);
  
  const Eigen::Affine3d& end_effector_state =
      kinematic_state->getGlobalLinkTransform("hand_link");  // TODO Add configuration parameter
  tf::poseEigenToMsg(end_effector_state, current_pose);  
//   debug_pos_pub_.publish(current_pose);

  tf::Quaternion q(current_pose.orientation.x, 
                   current_pose.orientation.y, 
                   current_pose.orientation.z, 
                   current_pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  currentPosition(0, 0) = current_pose.position.x;
  currentPosition(1, 0) = current_pose.position.y;
  currentPosition(2, 0) = current_pose.position.z;
  currentPosition(3, 0) = current_pose.orientation.x;
  currentPosition(4, 0) = current_pose.orientation.y;
  currentPosition(5, 0) = current_pose.orientation.z;
//   currentPosition(4, 0) = roll;
//   currentPosition(3, 0) = pitch;
//   currentPosition(5, 0) = yaw;
  
  prevPosition = currentPosition;

  cartesianPosition(0, 0) = 0.0;
  cartesianPosition(1, 0) = 0.0;
  cartesianPosition(2, 0) = 0.0;
  cartesianPosition(3, 0) = 0.0;
  cartesianPosition(4, 0) = 0.0;
  cartesianPosition(5, 0) = 0.0;

  for(int i=0; i<6; i++)
  {
    request_update_constraint[i] = true;
    request_update_constraint_tolerance[i] = 0.0;
  }
  UpdateAxisConstraints();
}

void InverseKinematic::ResolveInverseKinematic(double (&joint_position_command)[6],
                                               sensor_msgs::JointState& current_joint_state,
                                               double (&cartesian_velocity_desired)[6])
{
  ROS_WARN_STREAM("InverseKinematic::ResolveInverseKinematic");
  sensor_msgs::JointState local_joint_state = current_joint_state;
  kinematic_state->setVariableValues(local_joint_state);
//   for (std::size_t i = 0; i < local_joint_state.name.size(); ++i)
//   {
//     local_joint_state.position[i] = joint_position_command[i];
//   }
  
  const Eigen::Affine3d& end_effector_state =
      kinematic_state->getGlobalLinkTransform("hand_link");  // TODO Add configuration parameter
  tf::poseEigenToMsg(end_effector_state, current_pose);  
//   debug_pos_pub_.publish(current_pose);
  tf::Quaternion q(current_pose.orientation.x, 
                   current_pose.orientation.y, 
                   current_pose.orientation.z, 
                   current_pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
      
  currentPosition(0, 0) = current_pose.position.x;
  currentPosition(1, 0) = current_pose.position.y;
  currentPosition(2, 0) = current_pose.position.z;
  currentPosition(3, 0) = current_pose.orientation.x;
  currentPosition(4, 0) = current_pose.orientation.y;
  currentPosition(5, 0) = current_pose.orientation.z;
//   currentPosition(4, 0) = roll;
//   currentPosition(3, 0) = pitch;
//   currentPosition(5, 0) = yaw;

  for(int i=3; i<6;i++)
  {
    ROS_DEBUG_STREAM("delta=" << currentPosition(i, 0) - prevPosition(i, 0) <<"");

    if(std::abs(currentPosition(i, 0) - prevPosition(i, 0)) >= M_PI/2)
    {
      ROS_ERROR_STREAM("==============> Euler jump detected !!!! delta=" << currentPosition(i, 0) - prevPosition(i, 0) <<"");
      RequestUpdateAxisConstraints(i);
    }
  }
  prevPosition = currentPosition;
  
  UpdateAxisConstraints();
  
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);  
  Eigen::MatrixXd jacobian;
  if(!kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian))
  {
    ROS_ERROR_STREAM("JACOBIAN COMPUTATION ISSUE");
    exit(0);
  }
  ROS_DEBUG_STREAM("Jacobian: \n" << jacobian << "\n");

  const Vector6d dx_des_vect = Vector6d(cartesian_velocity_desired);
  ROS_DEBUG_STREAM("dx_des_vect: \n" << dx_des_vect << "\n");

  Matrix6d hessian = (jacobian.transpose() * alpha_weight * jacobian) + beta_weight;
  ROS_DEBUG_STREAM("hessian: \n" << hessian << "\n");

  
  
  Vector6d g = (-jacobian.transpose() * alpha_weight * dx_des_vect);
  ROS_DEBUG_STREAM("g = \n" << g << "\n");

  // control max and min velocity of joint (dq)
  double maxVel = 0.8;
  double lb[] = { -maxVel, -maxVel, -maxVel, -maxVel, -maxVel, -maxVel };
  double ub[] = { +maxVel, +maxVel, +maxVel, +maxVel, +maxVel, +maxVel };

  // Taylor developpement
  // q = q0 + dq*T
  // inequality is :
  // lb < q < ub
  // lb < q0 + dq*T < ub
  // (lb-q0) < dq.T < (ub-q0)
//   xlimit_max = x_max_limit[3] - currentPosition(3, 0);
//   ylimit_max = x_max_limit[4] - currentPosition(4, 0);
//   zlimit_max = x_max_limit[5] - currentPosition(5, 0);
//   
//   xlimit_min = x_min_limit[3] - currentPosition(3, 0);
//   ylimit_min = x_min_limit[4] - currentPosition(4, 0);
//   zlimit_min = x_min_limit[5] - currentPosition(5, 0);
  
  xlimit_max = 0.505 - currentPosition(3, 0);
  ylimit_max = 0.505 - currentPosition(4, 0);
  zlimit_max = 0.505 - currentPosition(5, 0);
  
  xlimit_min = 0.495 - currentPosition(3, 0);
  ylimit_min = 0.495 - currentPosition(4, 0);
  zlimit_min = 0.495 - currentPosition(5, 0);
  
  
  Eigen::Matrix<double, 12, 6, Eigen::RowMajor> A;
  A.topLeftCorner(6, 6) = Eigen::MatrixXd::Identity(6, 6) * CALC_PERIOD;
  A.bottomLeftCorner(6, 6) = jacobian.topLeftCorner(6, 6) * CALC_PERIOD;

  double lbA[] = { (joints_limits_min[0] - local_joint_state.position[0]),
                   (joints_limits_min[1] - local_joint_state.position[1]),
                   (joints_limits_min[2] - local_joint_state.position[2]),
                   (joints_limits_min[3] - local_joint_state.position[3]),
                   (joints_limits_min[4] - local_joint_state.position[4]),
                   (joints_limits_min[5] - local_joint_state.position[5]),
                   x_min_limit[0] - currentPosition(0, 0),
                   x_min_limit[1] - currentPosition(1, 0),
                   x_min_limit[2] - currentPosition(2, 0),                  
//                    x_min_limit[3] - currentPosition(3, 0),
//                    x_min_limit[4] - currentPosition(4, 0),
//                    x_min_limit[5] - currentPosition(5, 0)

   xlimit_min,
   ylimit_min,
   zlimit_min,
    
  };
  double ubA[] = { (joints_limits_max[0] - local_joint_state.position[0]),
                   (joints_limits_max[1] - local_joint_state.position[1]),
                   (joints_limits_max[2] - local_joint_state.position[2]),
                   (joints_limits_max[3] - local_joint_state.position[3]),
                   (joints_limits_max[4] - local_joint_state.position[4]),
                   (joints_limits_max[5] - local_joint_state.position[5]),
                   x_max_limit[0] - currentPosition(0, 0),
                   x_max_limit[1] - currentPosition(1, 0),
                   x_max_limit[2] - currentPosition(2, 0),
//                    x_max_limit[3] - currentPosition(3, 0),
//                    x_max_limit[4] - currentPosition(4, 0),
//                    x_max_limit[5] - currentPosition(5, 0)

   xlimit_max,
   ylimit_max,
   zlimit_max,
    
  };

  ROS_DEBUG_STREAM("A = \n" << A << "\n");
    ROS_DEBUG_STREAM("currentPosition = {" << currentPosition(0, 0) << ", " << currentPosition(1, 0) << ", " << currentPosition(2, 0) << ", " << currentPosition(3, 0) << ", "
                               << currentPosition(4, 0) << ", " << currentPosition(5, 0) << "}");
  ROS_DEBUG_STREAM("dx_des_vect = {" << dx_des_vect[0] << ", " << dx_des_vect[1] << ", " << dx_des_vect[2] << ", " << dx_des_vect[3] << ", "
                               << dx_des_vect[4] << ", " << dx_des_vect[5] << "}");
  ROS_DEBUG_STREAM("x_min_limit = {" << x_min_limit[0] << ", " << x_min_limit[1] << ", " << x_min_limit[2] << ", " << x_min_limit[3] << ", "
                               << x_min_limit[4] << ", " << x_min_limit[5] << "}");
  ROS_DEBUG_STREAM("x_max_limit = {" << x_max_limit[0] << ", " << x_max_limit[1] << ", " << x_max_limit[2] << ", " << x_max_limit[3] << ", "
                               << x_max_limit[4] << ", " << x_max_limit[5] << "}");
  
  ROS_DEBUG_STREAM("lbA = {" << lbA[6] << ", " << lbA[7] << ", " << lbA[8] << ", " << lbA[9] << ", "
                               << lbA[10] << ", " << lbA[11] << "}");  
  ROS_DEBUG_STREAM("ubA = {" << ubA[6] << ", " << ubA[7] << ", " << ubA[8] << ", " << ubA[9] << ", "
                               << ubA[10] << ", " << ubA[11] << "}");

  // Solve first QP.
  qpOASES::real_t xOpt[6];
  qpOASES::int_t nWSR = 10;
  int qp_return = 0;
  if (start_flag)
  {
    qp_return = IK->init(hessian.data(), g.data(), A.data(), lb, ub, lbA, ubA, nWSR, 0);
    start_flag = false;
  }
  else
  {
    qp_return = IK->hotstart(hessian.data(), g.data(), A.data(), lb, ub, lbA, ubA, nWSR, 0);
  }

  if (qp_return == qpOASES::SUCCESSFUL_RETURN)
  {
    ROS_INFO_STREAM("qpOASES : succesfully return");

    // Get and print solution of first QP

    IK->getPrimalSolution(xOpt);
    double theta_tmp[6];
    theta_tmp[0] = local_joint_state.position[0] + xOpt[0] * CALC_PERIOD;
    theta_tmp[1] = local_joint_state.position[1] + xOpt[1] * CALC_PERIOD;
    theta_tmp[2] = local_joint_state.position[2] + xOpt[2] * CALC_PERIOD;
    theta_tmp[3] = local_joint_state.position[3] + xOpt[3] * CALC_PERIOD;
    theta_tmp[4] = local_joint_state.position[4] + xOpt[4] * CALC_PERIOD;
    theta_tmp[5] = local_joint_state.position[5] + xOpt[5] * CALC_PERIOD;

    bool limit_detected = false;
    // Check joints limits
    for (int i = 0; i < 6; i++)
    {
      if (theta_tmp[i] > joints_limits_max[i])
      {
        ROS_WARN_STREAM("joint_" << i + 1 << " max limit overshoot : " << theta_tmp[i] << " > " << joints_limits_max[i]);
        limit_detected = true;
      }
      else if (theta_tmp[i] < joints_limits_min[i])
      {
        ROS_WARN_STREAM("joint_" << i + 1 << " min limit overshoot : " << theta_tmp[i] << " < " << joints_limits_min[i]);
        limit_detected = true;
      }
    }      
    
    if (!limit_detected)
    {
      joint_position_command[0] = theta_tmp[0];
      joint_position_command[1] = theta_tmp[1];
      joint_position_command[2] = theta_tmp[2];
      joint_position_command[3] = theta_tmp[3];
      joint_position_command[4] = theta_tmp[4];
      joint_position_command[5] = theta_tmp[5];
    }
  }
  else
  {
    ROS_ERROR_STREAM("qpOASES : Failed !!!");
//     exit(0);
  }
  ROS_DEBUG_STREAM("joint_position_command: \n[" << joint_position_command[0] << ", " << joint_position_command[1]
                                                 << ", " << joint_position_command[2] << ", "
                                                 << joint_position_command[3] << ", " << joint_position_command[4]
                                                 << ", " << joint_position_command[5] << "]\n");

  Vector6d cartesianSpeed = Vector6d::Zero();
  cartesianSpeed = jacobian*Vector6d(xOpt);
  ROS_DEBUG_STREAM("cartesianSpeed: \n" << cartesianSpeed << "\n");
  cartesianPosition = cartesianPosition + cartesianSpeed*CALC_PERIOD;
  ROS_DEBUG_STREAM("cartesianPosition: \n" << cartesianPosition << "\n");
  Vector6d jqt = cartesianSpeed*CALC_PERIOD;
  ROS_DEBUG_STREAM("jqt: \n" << jqt << "\n");
  geometry_msgs::Pose pose_des;
  pose_des.position.x = desiredPosition(0, 0);
  pose_des.position.y = desiredPosition(1, 0);
  pose_des.position.z = desiredPosition(2, 0);
  pose_des.orientation.x = desiredPosition(3, 0);
  pose_des.orientation.y = desiredPosition(4, 0);
  pose_des.orientation.z = desiredPosition(5, 0);
  debug_pos_des_pub_.publish(pose_des);
}

void InverseKinematic::UpdateAxisConstraints()
{
  for(int i=0;i<6;i++)
  {
    if(request_update_constraint[i])
    {
      ROS_WARN_STREAM("Update axis " << i << " constraints with new tolerance of " << request_update_constraint_tolerance[i] << " m.");
      if(i<3)
      {
          x_min_limit[i] = currentPosition(i, 0) - request_update_constraint_tolerance[i];
          x_max_limit[i] = currentPosition(i, 0) + request_update_constraint_tolerance[i];
      }
      else
      {
        x_min_limit[i] = - request_update_constraint_tolerance[i];
        x_max_limit[i] = + request_update_constraint_tolerance[i];
      }
      request_update_constraint[i] = false;
    }
  }
}

// Keep last set tolerance
void InverseKinematic::RequestUpdateAxisConstraints(int axis)
{
  ROS_WARN_STREAM("Update axis " << axis << " constraints with the last tolerance set (" << request_update_constraint_tolerance[axis] << ")");
  request_update_constraint[axis] = true;
}
void InverseKinematic::RequestUpdateAxisConstraints(int axis, double tolerance)
{
  ROS_WARN_STREAM("Update axis " << axis << " constraints with new tolerance of " << tolerance << "");
  request_update_constraint[axis] = true;
  request_update_constraint_tolerance[axis] = tolerance;
}
}
