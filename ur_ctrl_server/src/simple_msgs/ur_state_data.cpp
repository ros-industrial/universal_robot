
#include "ur_ctrl_server/simple_msgs/ur_state_data.h"
#include "ur_ctrl_server/ur_ctrl_shared.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"

using industrial::byte_array::ByteArray;
using industrial::simple_message::SimpleMessage;
namespace CommTypes = industrial::simple_message::CommTypes;
namespace ReplyTypes = industrial::simple_message::ReplyTypes;

namespace ur
{

URStateData::URStateData(void)
{
  sequence = 0;
  for(int i=0;i<6;i++) {
    q_act[i] = 0;
    qd_act[i] = 0;
    i_act[i] = 0;
    q_des[i] = 0;
    qd_des[i] = 0;
    qdd_des[i] = 0;
    i_des[i] = 0;
    acc_x[i] = 0;
    acc_y[i] = 0;
    acc_z[i] = 0;
    moment_des[i] = 0;
    tcp_force[i] = 0;
    tcp_speed[i] = 0;
    tcp_wrench[i] = 0;
    tcp_pose[i] = 0;
    joint_mode_ids[i] = 0;
  }
  tcp_force_scalar = 0;
  tcp_power = 0;
  tcp_payload = 0;
  power = 0;
  robot_mode_id = -1;
  is_power_on_robot = false;
  is_security_stopped = false;
  is_emergency_stopped = false;
  is_extra_button_pressed = false;
  is_power_button_pressed = false;
  is_safety_signal_such_that_we_should_stop = false;
}

URStateData::~URStateData(void) {}

bool URStateData::fromSimpleMessage(SimpleMessage & msg_in)
{
  ByteArray data = msg_in.getData();
  if(data.unload(*this)) 
    return true;
  else {
    LOG_ERROR("Failed to init URStateData from SimpleMessage");
    return false;
  }
}

bool URStateData::toSimpleMessage(int msg_type, SimpleMessage & msg_out)
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

bool URStateData::load(ByteArray *buffer)
{
  for(int i=0;i<6;i++) {
    if(!buffer->load(q_act[i])                                 ||
       !buffer->load(qd_act[i])                                ||
       !buffer->load(i_act[i])                                 ||
       !buffer->load(q_des[i])                                 ||
       !buffer->load(qd_des[i])                                ||
       !buffer->load(qdd_des[i])                               ||
       !buffer->load(i_des[i])                                 ||
       !buffer->load(acc_x[i])                                 ||
       !buffer->load(acc_y[i])                                 ||
       !buffer->load(acc_z[i])                                 ||
       !buffer->load(moment_des[i])                            ||
       !buffer->load(tcp_force[i])                             ||
       !buffer->load(tcp_speed[i])                             ||
       !buffer->load(tcp_wrench[i])                            ||
       !buffer->load(tcp_pose[i])                              ||
       !buffer->load(joint_mode_ids[i])) {
      goto load_failure;
    }
  }
  if(!buffer->load(tcp_force_scalar)                           ||
     !buffer->load(tcp_power)                                  || 
     !buffer->load(tcp_payload)                                || 
     !buffer->load(power)                                      || 
     !buffer->load(robot_mode_id)                              ||
     !buffer->load(is_power_on_robot)                          ||
     !buffer->load(is_security_stopped)                        ||
     !buffer->load(is_emergency_stopped)                       ||
     !buffer->load(is_extra_button_pressed)                    ||
     !buffer->load(is_power_button_pressed)                    ||
     !buffer->load(is_safety_signal_such_that_we_should_stop)  || 
     !buffer->load(this->sequence)) {
    goto load_failure;
  }
  return true;
load_failure:
  LOG_ERROR("Failed to load UR state buffer");
  return false;
}

bool URStateData::unload(ByteArray *buffer)
{
  if(!buffer->unload(this->sequence)                            ||
     !buffer->unload(is_safety_signal_such_that_we_should_stop) ||
     !buffer->unload(is_power_button_pressed)                   ||
     !buffer->unload(is_extra_button_pressed)                   ||
     !buffer->unload(is_emergency_stopped)                      ||
     !buffer->unload(is_security_stopped)                       ||
     !buffer->unload(is_power_on_robot)                         ||
     !buffer->unload(robot_mode_id)                             ||
     !buffer->unload(power)                                     ||
     !buffer->unload(tcp_payload)                               ||
     !buffer->unload(tcp_power)                                 ||
     !buffer->unload(tcp_force_scalar)) {
    goto unload_failure;
  }
  for(int i=5;i>=0;i--) {
    if(!buffer->unload(joint_mode_ids[i])                       ||
       !buffer->unload(tcp_pose[i])                             ||
       !buffer->unload(tcp_wrench[i])                           ||
       !buffer->unload(tcp_speed[i])                            ||
       !buffer->unload(tcp_force[i])                            ||
       !buffer->unload(moment_des[i])                           ||
       !buffer->unload(acc_z[i])                                ||
       !buffer->unload(acc_y[i])                                ||
       !buffer->unload(acc_x[i])                                ||
       !buffer->unload(i_des[i])                                ||
       !buffer->unload(qdd_des[i])                              ||
       !buffer->unload(qd_des[i])                               ||
       !buffer->unload(q_des[i])                                ||
       !buffer->unload(i_act[i])                                ||
       !buffer->unload(qd_act[i])                               ||
       !buffer->unload(q_act[i])) {
      goto unload_failure;
    }
  }
  return true;
unload_failure:
  LOG_ERROR("Failed to unload UR state buffer");
  return false;
}

}

