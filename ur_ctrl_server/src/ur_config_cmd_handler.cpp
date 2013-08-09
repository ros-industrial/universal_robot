#include "ur_ctrl_server/ur_config_cmd_handler.h"

using industrial::smpl_msg_connection::SmplMsgConnection;

namespace ur
{

bool URConfigCommandHandler::init(int msg_type, SmplMsgConnection* connection, 
                                 URConfigCommand* ur_config_cmd)
{
  ur_config_cmd_ = ur_config_cmd;
  return MessageHandler::init(msg_type, connection);
}

bool URConfigCommandHandler::internalCB(SimpleMessage& in)
{
  if(!ur_config_cmd_tmp_.fromSimpleMessage(in)) {
    LOG_ERROR("Failed to load URConfigCommand from SimpleMessage");
    return false;
  }
  ur_config_cmd_->addCommand(&ur_config_cmd_tmp_);
  has_updated_ = true;
  return true;
}

}
