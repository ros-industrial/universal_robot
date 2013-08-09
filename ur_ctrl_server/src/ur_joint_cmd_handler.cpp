#include "ur_ctrl_server/ur_joint_cmd_handler.h"

using industrial::smpl_msg_connection::SmplMsgConnection;

namespace ur
{

bool URJointCommandHandler::init(int msg_type, SmplMsgConnection* connection, 
                                 URJointCommand* ur_jnt_cmd)
{
  ur_jnt_cmd_ = ur_jnt_cmd;
  has_updated_ = false;
  last_sequence_ = -1;
  return MessageHandler::init(msg_type, connection);
}

bool URJointCommandHandler::internalCB(SimpleMessage& in)
{
  if(!ur_jnt_cmd_->fromSimpleMessage(in)) {
    LOG_ERROR("Failed to load URJointCommand from SimpleMessage");
    return false;
  }
  if(ur_jnt_cmd_->sequence > last_sequence_) {
    last_sequence_ = ur_jnt_cmd_->sequence;
    has_updated_ = true;
  }
  return true;
}

}
