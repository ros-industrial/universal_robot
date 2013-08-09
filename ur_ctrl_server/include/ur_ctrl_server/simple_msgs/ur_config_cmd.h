#ifndef UR_CONFIG_CMD_H
#define UR_CONFIG_CMD_H

#include <vector>

#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "ur_ctrl_server/ur_ctrl_shared.h"

using industrial::simple_serialize::SimpleSerialize;
using industrial::simple_message::SimpleMessage;
using industrial::shared_types::shared_int;
using industrial::shared_types::shared_real;

namespace ur
{

struct URConfigCommand : public SimpleSerialize
{
  URConfigCommand(void);
  ~URConfigCommand(void);

  bool fromSimpleMessage(SimpleMessage & msg_in);
  bool toSimpleMessage(int msg_type, SimpleMessage & msg_out);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return ( 6 *  3 * sizeof(industrial::shared_types::shared_real)) +
           ( 1 *  1 * sizeof(industrial::shared_types::shared_real)) +
           ( 1 *  5 * sizeof(industrial::shared_types::shared_int));
  }

  // adds function calls from new_cmd to this command
  void addCommand(const URConfigCommand* new_cmd);
  // sets all function calls to nil
  void clearCommands();

  // UR configuration methods
  void openRealInterface();
  void openSimulatedInterface();
  void closeInterface();
  void unlockSecurityStop();
  void setRobotReadyMode();
  void setRobotRunningMode();
  void powerOnRobot();
  void powerOffRobot();
  void setTCP(const std::vector<double>& tcp_pose);
  void setTCPPayloadCOG(const std::vector<double>& tcp_payload_cog);
  void setTCPPayload(double tcp_payload);
  void setTCPWrenchEE(const std::vector<double>& tcp_wrench);
  void setTCPWrenchBase(const std::vector<double>& tcp_wrench);
  void setSecurityStop(char joint_code, int error_state, int error_argument);

  // stored data
  shared_int func_calls; // bit-mask for the different commands to call this loop

  // set_tcp_wrench(const double *new_tcp_wrench, const int in_base_coord)
  shared_real set_tcp_wrench[6];
  shared_int set_tcp_wrench_in_base_coord;

  // security_stop(char joint_code, int error_state, int error_argument)
  shared_int security_stop_joint_code;
  shared_int security_stop_error_state;
  shared_int security_stop_error_argument;

  // set_tcp(const double *tcp_pose)
  shared_real set_tcp_pose[6];
  // set_tcp_payload_cog(const double *tcp_payload_cog)
  shared_real set_tcp_payload_cog[6];
  // set_tcp_payload(double tcp_payload)
  shared_real set_tcp_payload;
};
}


#endif // UR_CONFIG_CMD_H
