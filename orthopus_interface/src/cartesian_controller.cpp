// TODO copyright
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>

#include <niryo_one_msgs/GetInt.h>

#include "orthopus_interface/cartesian_controller.h"

#define RATE 10
#define TOOL_ID_GRIPPER_1 11
#define TOOL_ID_GRIPPER_2 12
#define INIT_TIME 1

namespace cartesian_controller
{
CartesianController::CartesianController()
{
  ROS_INFO("CartesianController constructor");
  ros::Rate loop_rate = ros::Rate(RATE);

  joints_sub_ = n_.subscribe("joint_states", 1, &CartesianController::jointStatesCB, this);
  dx_des_sub_ = n_.subscribe("dx_des", 1, &CartesianController::dxDesCB, this);
  gripper_sub_ = n_.subscribe("gripper_des", 1, &CartesianController::gripperCB, this);

  command_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("/niryo_one_follow_joint_trajectory_controller/command",
                                                                1);  // TODO check optimal queue size
  debug_pub_ = n_.advertise<geometry_msgs::Pose>("/debug_cartesian_pos", 1);
  debug_des_pub_ = n_.advertise<geometry_msgs::Pose>("/debug_cartesian_pos_des", 1);


  ros::ServiceServer enable_service = n_.advertiseService("/niryo_one/joystick_interface/enable",
                                  &CartesianController::enableCB, this);
  
  tool_controller_.setToolId(TOOL_ID_GRIPPER_2);
  // Wait for initial messages
  ROS_INFO("Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_INFO("Received first joint msg.");

  enable_joy_ = false;
  
  // Initialize attributes
  for (int i = 0; i < 6; i++)
  {
    joint_position_cmd[i] = 0.0;
    cartesian_velocity_desired[i] = 0.0;
  }

  ROS_INFO("Initialize robot position");
  // initialize robot in zero position
  while (ros::ok())
  {
    ros::spinOnce();
    sendInitCommand();
    ros::Duration(INIT_TIME + 1.0).sleep();
    break;
  }

  ros::spinOnce();
  ik_ = new InverseKinematic(debug_pub_, debug_des_pub_);
  ik_->Init(current_joint_state);

  while (ros::ok())
  {
    ros::spinOnce();
    if (enable_joy_)
    {
      ROS_INFO("=== Start IK computation...");
      ik_->ResolveInverseKinematic(joint_position_cmd, current_joint_state, cartesian_velocity_desired);
      ROS_INFO("    Done.");

      ROS_INFO("=== Send Niryo One commands...");
      send6DofCommand();
      tool_controller_.sendGripperCommand(gripper_state_);
      ROS_INFO("    Done.");
    }
    loop_rate.sleep();
  }
}

void CartesianController::sendInitCommand()
{
  trajectory_msgs::JointTrajectory new_jt_traj;

  new_jt_traj.header.stamp = ros::Time::now();
  new_jt_traj.joint_names = current_joint_state.name;
  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(INIT_TIME);
  point.positions = current_joint_state.position;

  for (int i = 0; i < 6; i++)
  {
    point.positions[i] = 0;
  }

  // Important : do not send velocity as cubic interpolation is done !
  // It should be better to handle that by proper velocity command (force velocity ramp on xbox controller)
  // point.velocities = current_joint_state.velocity;
  new_jt_traj.points.push_back(point);
  command_pub_.publish(new_jt_traj);
};

void CartesianController::send6DofCommand()
{
  if (enable_joy_)
  {
    trajectory_msgs::JointTrajectory new_jt_traj;
    new_jt_traj.header.stamp = ros::Time::now();
    new_jt_traj.joint_names = current_joint_state.name;

    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(1.0 / RATE);
    for (int i = 0; i < 6; i++)
    {
      point.positions.push_back(joint_position_cmd[i]);
    }

    // Important : do not send velocity as cubic interpolation is done !
    // It should be better to handle that by proper velocity command (force velocity ramp on xbox controller)
    // point.velocities = current_joint_state.velocity;
    new_jt_traj.points.push_back(point);
    command_pub_.publish(new_jt_traj);
  }
};

// Scale the incoming desired velocity
Vector6d CartesianController::scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const
{
  Vector6d result;

  result(0) = command.twist.linear.x;
  result(1) = command.twist.linear.y;
  result(2) = command.twist.linear.z;
  result(3) = command.twist.angular.x;
  result(4) = command.twist.angular.y;
  result(5) = command.twist.angular.z;

  return result;
}

// Convert incoming joy commands to TwistStamped commands for jogging.
// The TwistStamped component goes to jogging, while buttons 0 & 1 control
// joints directly.
void CartesianController::jointStatesCB(const sensor_msgs::JointStateConstPtr& msg)
{
  current_joint_state = *msg;
}

void CartesianController::dxDesCB(const geometry_msgs::TwistStampedPtr& msg)
{
//   ROS_DEBUG_STREAM("dxDesCB");
  for(int i=0; i<6;i++)
  {
    cartesian_velocity_desired[i] = 0.0;
  }
  
  if (enable_joy_)
  {
    cartesian_velocity_desired[0] = msg->twist.linear.x;
    cartesian_velocity_desired[1] = msg->twist.linear.y;
    cartesian_velocity_desired[2] = msg->twist.linear.z;

    cartesian_velocity_desired[3] = msg->twist.angular.x;
    cartesian_velocity_desired[4] = msg->twist.angular.y;
    cartesian_velocity_desired[5] = msg->twist.angular.z;

    for(int i =0; i<3;i++)
    {
      if(cartesian_velocity_desired[i] != 0)
      {
        ik_->RequestUpdateAxisConstraints(i, 1.0);
      }
      else if(cartesian_velocity_desired[i] == 0 && cartesian_velocity_desired_prev[i] != 0)
      {
        ik_->RequestUpdateAxisConstraints(i, 0.001);
      }
      cartesian_velocity_desired_prev[i] = cartesian_velocity_desired[i];
    }
//     for(int i =3; i<6;i++)
//     {
//       if(cartesian_velocity_desired[i] != 0)
//       {
//         if/*(i==3)
//         {
//           ik_->RequestUpdateAxisConstraints(5,1.0);
//         }
//         else if(i==4)
//         {
//           ik_->RequestUpdateAxisConstraints(3,1.0);
//         }        
//         else if(i==5)
//         {
//           ik_->RequestUpdateAxisConstraints(4,1.0);
//         }*/
//         ik_->RequestUpdateAxisConstraints(i,1.0);
//       }
//       else if(cartesian_velocity_desired[i] == 0 && cartesian_velocity_desired_prev[i] != 0)
//       {
// //         if(i==3)
// //         {
// //         ik_->RequestUpdateAxisConstraints(5, 0.0);
// //         }
// //         else if(i==4)
// //         {
// //         ik_->RequestUpdateAxisConstraints(3, 0.0);
// //         }
// //         else if(i==5)
// //         {
// //         ik_->RequestUpdateAxisConstraints(4, 0.0);
// //         }
//         ik_->RequestUpdateAxisConstraints(i, 0.0);
//       }
//       cartesian_velocity_desired_prev[i] = cartesian_velocity_desired[i];
//     }
  }
}

void CartesianController::gripperCB(const std_msgs::BoolPtr& msg)
{
    gripper_state_ = msg->data;
}

bool CartesianController::enableCB(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res)
{
  std::string result_message = "";
  int result = 0;
  if (req.value == 1)
  {
    if (can_enable())
    {
      enable_joy();
      result = 200;
      result_message = "Cartesian control has been enabled";
    }
    else
    {
      result = 400;
      result_message = "Wait for the end of command to enable the cartesian control";
    }
  }
  else
  {
    disable_joy();
    result = 200;
    result_message = "Cartesian control has been disabled";
  }
  res.status = result;
  res.message = result_message;
  return true;
}

bool CartesianController::can_enable()
{
  ros::service::waitForService("/niryo_one/commander/is_active");
  niryo_one_msgs::GetInt is_active;
  ros::ServiceClient commander_active_client =
      n_.serviceClient<niryo_one_msgs::GetInt>("/niryo_one/commander/is_active");
  commander_active_client.call(is_active);
  return (is_active.response.value == 0);
}

void CartesianController::enable_joy()
{
  ik_->Init(current_joint_state);
  for(int i=0; i<7; i++)
  {
    joint_position_cmd[i] = current_joint_state.position[i];
  }
  enable_joy_ = true;
}

void CartesianController::disable_joy()
{
  // TODO handle current motion : wait the arm reach a stable state
  enable_joy_ = false;
}

}

using namespace cartesian_controller;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_controller");

  CartesianController cartesian_controller;

  return 0;
}
