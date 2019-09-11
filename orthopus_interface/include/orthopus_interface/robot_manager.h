/*
  TODO copyright
*/
#ifndef CARTESIAN_CONTROLLER_ROBOT_MANAGER_H
#define CARTESIAN_CONTROLLER_ROBOT_MANAGER_H

#include "ros/ros.h"

// #include "sensor_msgs/JointState.h"
#include "orthopus_interface/pose_manager.h"
#include "orthopus_interface/cartesian_controller.h"

namespace cartesian_controller
{
typedef actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction> NiryoClient;
class RobotManager
{
public:
  RobotManager();
  void init(sensor_msgs::JointState& current_joint_state);
  
protected:
private:
  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();
  void initializeActions();
  
  ros::NodeHandle n_;
  
  ros::Publisher command_pub_;
  ros::Publisher joystick_enabled_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher debug_des_pub_;
  
  ros::Subscriber joints_sub_;
  ros::Subscriber move_group_state_;
  ros::Subscriber dx_des_sub_;
  ros::Subscriber learning_mode_sub_;
  
  ros::ServiceServer action_service_;
  ros::ServiceServer cartesian_enable_service_;
  ros::ServiceServer manage_pose_service_;
    
  PoseManager pose_manager_;
  CartesianController cartesian_controller_;
  
};

}
#endif
 
