#ifndef UR_MSG_HANDLERS_H
#define UR_MSG_HANDLERS_H

#include "ros/ros.h"

#include "simple_message/smpl_msg_connection.h"
#include "simple_message/message_handler.h"
#include "ur_ctrl_server/simple_msgs/ur_state_data.h"

using industrial::message_handler::MessageHandler;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;

namespace ur
{

class URStateMsgHandler : public MessageHandler
{
private:
  ros::NodeHandle nh_;

  SmplMsgConnection* connection_;

  bool internalCB(industrial::simple_message::SimpleMessage & in);
  bool has_updated_;
  URStateData* ur_state_;

public:
  bool init(SmplMsgConnection* connection, URStateData* ur_state);
  bool hasUpdated() { return has_updated_; }
  bool reset() { has_updated_ = false; }
};

}

#endif // UR_MSG_HANDLERS_H
