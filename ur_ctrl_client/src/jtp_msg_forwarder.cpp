
#include "ur_ctrl_client/jtp_msg_forwarder.h"

namespace industrial
{

bool JTPMessageForwarder::init(SmplMsgConnection* connection)
{
  connection_ = connection;
  cmd_sub_ = nh_.subscribe("input", 1, &JTPMessageForwarder::cmdCallback, this);
  return true;
}

void JTPMessageForwarder::cmdCallback(const ROSJointTrajPtFull::ConstPtr& msg)
{
  if(!connection_->isConnected())
    return;

  for(int i=0;i<10;i++) {
    positions_.setJoint(i, msg->positions[i]);
    velocities_.setJoint(i, msg->velocities[i]);
    accelerations_.setJoint(i, msg->accelerations[i]);
  }
  jtp_.init(msg->robot_id, msg->sequence, msg->valid_fields, msg->time, 
            positions_, velocities_, accelerations_);
  // TODO msgType commtype, replycode, jtp_
  jtp_msg_.init(jtp_);
  jtp_msg_.toTopic(simp_msg_);
  connection_->sendMsg(simp_msg_);
  printf("sup\n");
}

}
