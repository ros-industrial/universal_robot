#ifndef JTP_MSG_FORWARDER_H
#define JTP_MSG_FORWARDER_H

#include "ros/ros.h"

#include "simple_message/smpl_msg_connection.h"
#include "simple_message/joint_traj_pt_full.h"
#include "simple_message/messages/joint_traj_pt_full_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/message_handler.h"

#ifdef FLOAT64
#include "ur_ctrl_client/JointTrajPtFull64.h"
typedef ur_ctrl_client::JointTrajPtFull64 ROSJointTrajPtFull;
#else
#include "ur_ctrl_client/JointTrajPtFull32.h"
typedef ur_ctrl_client::JointTrajPtFull32 ROSJointTrajPtFull;
#endif

using industrial::message_handler::MessageHandler;
using industrial::joint_traj_pt_full::JointTrajPtFull;
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
using industrial::joint_data::JointData;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;

namespace industrial
{

// Simply forwards a JointTrajPtFull message from ROS through simple_message
class JTPMessageForwarder
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;

  SmplMsgConnection* connection_;

  JointTrajPtFull jtp_;
  JointTrajPtFullMessage jtp_msg_;
  SimpleMessage simp_msg_;
  JointData positions_;
  JointData velocities_;
  JointData accelerations_;

public:
  bool init(SmplMsgConnection* connection);
  void cmdCallback(const ROSJointTrajPtFull::ConstPtr& msg);
};

}

#endif // JTP_MSG_FORWARDER_H
