#ifndef UR_JOINT_CMD_HANDLER_H
#define UR_JOINT_CMD_HANDLER_H

#include "simple_message/message_handler.h"
#include "simple_message/log_wrapper.h"
#include "ur_ctrl_server/simple_msgs/ur_joint_cmd.h"

using industrial::message_handler::MessageHandler;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;

namespace ur
{

struct URJointCommandHandler : public MessageHandler
{
public:
  URJointCommandHandler() {}
  ~URJointCommandHandler() {}
  bool init(int msg_type, SmplMsgConnection* connection, URJointCommand* ur_jnt_cmd);
  URJointCommand* ur_jnt_cmd_;
  bool hasUpdated() {return has_updated_;}
  void reset() {has_updated_ = false;}
  int last_sequence_;
private:
  bool internalCB(SimpleMessage& in);
  bool has_updated_;
};

}
#endif // UR_JOINT_CMD_HANDLER_H
