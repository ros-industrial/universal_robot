

#ifndef UR_ROBOT_HW_H
#define UR_ROBOT_HW_H

#define ROS
#define LINUXSOCKETS
#define FLOAT64

#include "simple_message/simple_message.h"
#include "simple_message/message_manager.h"
#ifdef TCP_COM
#include "simple_message/socket/tcp_client.h"
typedef industrial::tcp_client::TcpClient SimpleClient;
#else
#include "simple_message/socket/udp_client.h"
typedef industrial::udp_client::UdpClient SimpleClient;
#endif

#include "realtime_tools/realtime_publisher.h"
#include "hardware_interface/robot_hw.h"

#include "ur_ctrl_client/pos_vel_acc_joint_iface.h"
#include "ur_ctrl_client/ur_torque_joint_iface.h"
#include "ur_ctrl_client/joint_mode_iface.h"
#include "ur_ctrl_client/ur_config_iface.h"

#include "ur_ctrl_server/simple_msgs/ur_joint_cmd.h"
#include "ur_ctrl_server/simple_msgs/ur_config_cmd.h"
#include "ur_ctrl_server/ur_ctrl_shared.h"
#include "ur_ctrl_server/simple_msgs/ur_state_data.h"
#include "ur_ctrl_client/ur_msg_handlers.h"

using industrial::simple_message::SimpleMessage;
using industrial::message_manager::MessageManager;

using hardware_interface::JointStateInterface;
using hardware_interface::PosVelAccJointInterface;
using hardware_interface::VelocityJointInterface;
using hardware_interface::JointStateHandle;
using hardware_interface::PosVelAccJointHandle;
using hardware_interface::JointHandle;
using hardware_interface::JointModeInterface;
using hardware_interface::JointModeHandle;

namespace ur
{

class URRobotHW : public hardware_interface::RobotHW
{
public:
  URRobotHW(ros::NodeHandle& nh, std::vector<std::string>& joint_names);

  void init(std::string& robot_ip);
  void clearCommands();
  void read();
  void write();

private:

  ros::NodeHandle nh_;

  // simple_message communication objects
  SimpleClient connection_;
  MessageManager msg_man_;
  URStateMsgHandler ur_state_hdl_;

  // UR message objects
  URJointCommand ur_joint_cmd_;
  SimpleMessage ur_joint_cmd_msg_;
  URConfigCommand ur_config_cmd_;
  SimpleMessage ur_config_cmd_msg_;
  URStateData ur_state_data_;

  // ros_control interfaces
  JointStateInterface     jnt_state_iface_;
  PosVelAccJointInterface jnt_pos_vel_acc_iface_;
  VelocityJointInterface  jnt_vel_iface_;
  URTorqueJointInterface  jnt_torque_iface_;
  JointModeInterface      jnt_mode_iface_;
  URConfigInterface       config_iface_;
  
  boost::recursive_mutex config_lock_;

  // the current joint command mode
  int joint_mode_;

  // pos/vel/acc control
  double pva_pos_cmd_[6]; 
  double pva_vel_cmd_[6]; 
  double pva_acc_cmd_[6];

  // velocity only control
  double vel_cmd_[6]; 

  // torque control
  double torque_cmd_[6];
  double torque_vel_cmd_[6]; 
  double secu_torque_cmd_[6];
  double softness_cmd_[6];

};
}


#endif
