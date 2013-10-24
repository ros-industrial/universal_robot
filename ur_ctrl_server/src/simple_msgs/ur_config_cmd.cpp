
#include "ur_ctrl_server/simple_msgs/ur_config_cmd.h"
#include "ur_ctrl_server/ur_ctrl_shared.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"

using industrial::byte_array::ByteArray;
using industrial::simple_message::SimpleMessage;
using namespace industrial::simple_message;

namespace ur
{

URConfigCommand::URConfigCommand(void)
{
  func_calls = 0;
}

URConfigCommand::~URConfigCommand(void) {}

bool URConfigCommand::fromSimpleMessage(SimpleMessage & msg_in)
{
  ByteArray data = msg_in.getData();
  if(data.unload(*this)) 
    return true;
  else {
    LOG_ERROR("Failed to init URConfigCommand from SimpleMessage");
    return false;
  }
}

bool URConfigCommand::toSimpleMessage(int msg_type, SimpleMessage & msg_out)
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

bool URConfigCommand::load(ByteArray *buffer)
{
  if(!buffer->load(func_calls) ||
     !buffer->load(set_tcp_wrench_in_base_coord) ||
     !buffer->load(security_stop_joint_code) ||
     !buffer->load(security_stop_error_state) ||
     !buffer->load(security_stop_error_argument) ||
     !buffer->load(set_tcp_payload)) {
    goto load_failure;
  }
  for(int i=0;i<6;i++) {
    if(!buffer->load(set_tcp_wrench[i])   ||
       !buffer->load(set_tcp_pose[i])  ||
       !buffer->load(set_tcp_payload_cog[i])) {
      goto load_failure;
    }
  }
  return true;
load_failure:
  LOG_ERROR("Failed to load UR config command buffer");
  return false;
}

bool URConfigCommand::unload(ByteArray *buffer)
{
  for(int i=5;i>=0;i--) {
    if(!buffer->unload(set_tcp_payload_cog[i])   ||
       !buffer->unload(set_tcp_pose[i])  ||
       !buffer->unload(set_tcp_wrench[i])) {
      goto unload_failure;
    }
  }
  if(!buffer->unload(set_tcp_payload) ||
     !buffer->unload(security_stop_error_argument) ||
     !buffer->unload(security_stop_error_state) ||
     !buffer->unload(security_stop_joint_code) ||
     !buffer->unload(set_tcp_wrench_in_base_coord) ||
     !buffer->unload(func_calls)) {
    goto unload_failure;
  }
  return true;
unload_failure:
  LOG_ERROR("Failed to unload UR config command buffer");
  return false;
}

void URConfigCommand::addCommand(const URConfigCommand* new_cmd)
{
  func_calls |= new_cmd->func_calls;
  if(new_cmd->func_calls & URI_SET_TCP_WRENCH) {
    for(int i=0;i<6;i++) set_tcp_wrench[i] = new_cmd->set_tcp_wrench[i];
    set_tcp_wrench_in_base_coord = new_cmd->set_tcp_wrench_in_base_coord;
  }
  if(new_cmd->func_calls & URI_SET_SECURITY_STOP) {
    security_stop_joint_code = new_cmd->security_stop_joint_code;
    security_stop_error_state = new_cmd->security_stop_error_state;
    security_stop_error_argument = new_cmd->security_stop_error_argument;
  }
  if(new_cmd->func_calls & URI_SET_TCP) {
    for(int i=0;i<6;i++) set_tcp_pose[i] = new_cmd->set_tcp_pose[i];
  }
  if(new_cmd->func_calls & URI_SET_TCP_PAYLOAD_COG) {
    for(int i=0;i<6;i++) set_tcp_payload_cog[i] = new_cmd->set_tcp_payload_cog[i];
  }
  if(new_cmd->func_calls & URI_SET_TCP_PAYLOAD_COG) {
    set_tcp_payload = new_cmd->set_tcp_payload;
  }
}
  /////////////////////////////// Config commands ///////////////////////////////
  
void URConfigCommand::openRealInterface()
{
  func_calls |= URI_OPEN_REAL;
}

void URConfigCommand::openSimulatedInterface()
{
  func_calls |= URI_OPEN_SIMULATED;
}

void URConfigCommand::closeInterface()
{
  func_calls |= URI_CLOSE;
}

void URConfigCommand::unlockSecurityStop()
{
  func_calls |= URI_UNLOCK_SECURITY_STOP;
}

void URConfigCommand::setRobotReadyMode()
{
  func_calls |= URI_SET_ROBOT_READY_MODE;
}

void URConfigCommand::setRobotRunningMode()
{
  func_calls |= URI_SET_ROBOT_RUNNING_MODE;
}

void URConfigCommand::setFreedriveMode()
{
  func_calls |= URI_SET_FREEDRIVE_MODE;
}

void URConfigCommand::powerOnRobot()
{
  func_calls |= URI_POWER_ON_ROBOT;
}

void URConfigCommand::powerOffRobot()
{
  func_calls |= URI_POWER_OFF_ROBOT;
}

void URConfigCommand::setTCP(const std::vector<double>& tcp_pose)
{
  if(tcp_pose.size() != 6) {
    LOG_WARN("URConfigCommand: TCP pose must be of length 6");
    return;
  }
  std::copy(tcp_pose.begin(), tcp_pose.end(), set_tcp_pose); 
  func_calls |= URI_SET_TCP;
}

void URConfigCommand::setTCPPayloadCOG(const std::vector<double>& tcp_payload_cog)
{
  if(tcp_payload_cog.size() != 6) {
    LOG_WARN("URConfigCommand: TCP payload COG must be of length 6");
    return;
  }
  std::copy(tcp_payload_cog.begin(), tcp_payload_cog.end(), set_tcp_payload_cog); 
  func_calls |= URI_SET_TCP_PAYLOAD_COG;
}

void URConfigCommand::setTCPPayload(double tcp_payload)
{
  set_tcp_payload = tcp_payload;
  func_calls |= URI_SET_TCP_PAYLOAD;
}

void URConfigCommand::setTCPWrenchEE(const std::vector<double>& tcp_wrench)
{
  if(tcp_wrench.size() != 6) {
    LOG_WARN("URConfigCommand: TCP wrench must be of length 6");
    return;
  }
  std::copy(tcp_wrench.begin(), tcp_wrench.end(), set_tcp_wrench); 
  set_tcp_wrench_in_base_coord = 0;
  func_calls |= URI_SET_TCP_WRENCH;
}

void URConfigCommand::setTCPWrenchBase(const std::vector<double>& tcp_wrench)
{
  if(tcp_wrench.size() != 6) {
    LOG_WARN("URConfigCommand: TCP wrench must be of length 6");
    return;
  }
  std::copy(tcp_wrench.begin(), tcp_wrench.end(), set_tcp_wrench); 
  set_tcp_wrench_in_base_coord = 1;
  func_calls |= URI_SET_TCP_WRENCH;
}

void URConfigCommand::setSecurityStop(char joint_code, int error_state, int error_argument)
{
  security_stop_joint_code = joint_code;
  security_stop_error_state = error_state;
  security_stop_error_argument = error_argument;
  func_calls |= URI_SET_SECURITY_STOP;
}

void URConfigCommand::clearCommands() {func_calls = 0;}

}

