#ifndef UR_CONFIG_CMD_HANDLER_H
#define UR_CONFIG_CMD_HANDLER_H

#include "simple_message/message_handler.h"
#include "simple_message/log_wrapper.h"
#include "ur_ctrl_server/simple_msgs/ur_config_cmd.h"

using industrial::message_handler::MessageHandler;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;

namespace ur
{

struct URConfigCommandHandler : public MessageHandler
{
public:
  URConfigCommandHandler() {}
  ~URConfigCommandHandler() {}
  bool init(int msg_type, SmplMsgConnection* connection, URConfigCommand* ur_config_cmd);
  bool hasUpdated() {return has_updated_;}
  void reset() {has_updated_ = false; ur_config_cmd_->clearCommands(); }
  URConfigCommand* ur_config_cmd_;
private:
  URConfigCommand ur_config_cmd_tmp_;
  bool internalCB(SimpleMessage& in);
  bool has_updated_;
};

}
#endif // UR_CONFIG_CMD_HANDLER_H
