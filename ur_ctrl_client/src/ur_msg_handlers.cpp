
#include "ur_ctrl_client/ur_msg_handlers.h"
#include "ur_ctrl_server/ur_ctrl_shared.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::simple_message;

namespace ur
{

bool URStateMsgHandler::init(SmplMsgConnection* connection, URStateData* ur_state)
{
  has_updated_ = false;
  ur_state_ = ur_state;
  return MessageHandler::init(ur::URMessageTypes::STATE, connection);
}

bool URStateMsgHandler::internalCB(industrial::simple_message::SimpleMessage & in)
{
  if(!ur_state_->fromSimpleMessage(in)) {
    LOG_ERROR("Failed to initialize UR state message");
    return false;
  }
  has_updated_ = true;
  return true;
}

}
