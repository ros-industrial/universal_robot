
#include "ur_ctrl_server/simple_msgs/ur_joint_cmd.h"
#include "ur_ctrl_server/ur_ctrl_shared.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"

using industrial::byte_array::ByteArray;
using industrial::simple_message::SimpleMessage;
using namespace industrial::simple_message;

namespace ur
{

URJointCommand::URJointCommand(void)
{
  mode = 0;
  sequence = 0;
  for(int i=0;i<6;i++) {
    q[i] = 0;
    qd[i] = 0;
    qdd[i] = 0;
    softness[i] = 0;
  }
}

URJointCommand::~URJointCommand(void) {}

bool URJointCommand::fromSimpleMessage(SimpleMessage & msg_in)
{
  ByteArray data = msg_in.getData();
  if(data.unload(*this)) 
    return true;
  else {
    LOG_ERROR("Failed to init URJointCommand from SimpleMessage");
    return false;
  }
}

bool URJointCommand::toSimpleMessage(int msg_type, SimpleMessage & msg_out)
{
  ByteArray data;
  if(!data.load(*this)) {
    LOG_ERROR("Falied to load data into ByteArray");
    return false;
  }
  if(!msg_out.init(msg_type, CommTypes::TOPIC, ReplyTypes::INVALID, data)) {
    LOG_ERROR("Failed to init SimpleMessage");
  }
  return true;
}

bool URJointCommand::load(ByteArray *buffer)
{
  if(!buffer->load(mode) ||
     !buffer->load(sequence)) {
    goto load_failure;
  }
  for(int i=0;i<6;i++) {
    if(!buffer->load(q[i])   ||
       !buffer->load(qd[i])  ||
       !buffer->load(qdd[i]) ||
       !buffer->load(softness[i])) {
      goto load_failure;
    }
  }
  return true;
load_failure:
  LOG_ERROR("Failed to load UR joint command buffer");
  return false;
}

bool URJointCommand::unload(ByteArray *buffer)
{
  for(int i=5;i>=0;i--) {
    if(!buffer->unload(softness[i]) ||
       !buffer->unload(qdd[i])      ||
       !buffer->unload(qd[i])       ||
       !buffer->unload(q[i])) {
      goto unload_failure;
    }
  }
  if(!buffer->unload(sequence) ||
     !buffer->unload(mode)) {
    goto unload_failure;
  }
  return true;
unload_failure:
  LOG_ERROR("Failed to unload UR joint command buffer");
  return false;
}

}

