
#ifndef UR_JOINT_CMD_H
#define UR_JOINT_CMD_H

#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "ur_ctrl_server/ur_ctrl_shared.h"

using industrial::simple_serialize::SimpleSerialize;
using industrial::simple_message::SimpleMessage;
using industrial::shared_types::shared_int;
using industrial::shared_types::shared_real;
using industrial::shared_types::shared_bool;

namespace ur
{

struct URJointCommand : public SimpleSerialize
{
  URJointCommand(void);
  ~URJointCommand(void);

  bool fromSimpleMessage(SimpleMessage & msg_in);
  bool toSimpleMessage(int msg_type, SimpleMessage & msg_out);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return ( 6 *  4 * sizeof(industrial::shared_types::shared_real)) +
           ( 1 *  2 * sizeof(industrial::shared_types::shared_int));
  }

  shared_int mode;
  shared_int sequence;

  /////////////////////////// Joint Command Params //////////////////////////
  union {
    shared_real q[6];
    shared_real security_torque[6];
  };
  shared_real qd[6];
  union {
    shared_real qdd[6];
    shared_real control_torque[6];
  };
  shared_real softness[6];
  ///////////////////////////////////////////////////////////////////////////
};
}


#endif // UR_JOINT_CMD_H
