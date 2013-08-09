
#ifndef UR_STATE_DATA_H
#define UR_STATE_DATA_H

#include <vector>

#include <simple_message/simple_message.h>
#include <simple_message/simple_serialize.h>
#include <simple_message/shared_types.h>

using industrial::simple_serialize::SimpleSerialize;
using industrial::simple_message::SimpleMessage;
using industrial::shared_types::shared_int;
using industrial::shared_types::shared_real;
using industrial::shared_types::shared_bool;

namespace ur
{

struct URStateData : public SimpleSerialize
{
  URStateData(void);
  ~URStateData(void);

  bool fromSimpleMessage(SimpleMessage & msg_in);
  bool toSimpleMessage(int msg_type, SimpleMessage & msg_out);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return ( 6 * 15 * sizeof(industrial::shared_types::shared_real)) +
           ( 6 *  1 * sizeof(industrial::shared_types::shared_int)) +
           ( 1 *  4 * sizeof(industrial::shared_types::shared_real)) +
           ( 1 *  2 * sizeof(industrial::shared_types::shared_int)) +
           ( 1 *  6 * sizeof(industrial::shared_types::shared_bool));
  }

  /////////////////////////////// Accessors /////////////////////////////////

  int getSequence() { return sequence; }

  /////////////////////////// Robot Joint States ////////////////////////////

  void getActualQ(std::vector<double>& out) { std::copy(q_act, q_act+6, out.begin()); }
  void getActualQD(std::vector<double>& out) { std::copy(qd_act, qd_act+6, out.begin()); }
  void getActualI(std::vector<double>& out) { std::copy(i_act, i_act+6, out.begin()); }
  void getDesiredQ(std::vector<double>& out) { std::copy(q_des, q_des+6, out.begin()); }
  void getDesiredQD(std::vector<double>& out) { std::copy(qd_des, qd_des+6, out.begin()); }
  void getDesiredQDD(std::vector<double>& out) { std::copy(qdd_des, qdd_des+6, out.begin()); }
  void getDesiredI(std::vector<double>& out) { std::copy(i_des, i_des+6, out.begin()); }
  void getAccelX(std::vector<double>& out) { std::copy(acc_x, acc_x+6, out.begin()); }
  void getAccelY(std::vector<double>& out) { std::copy(acc_y, acc_y+6, out.begin()); }
  void getAccelZ(std::vector<double>& out) { std::copy(acc_z, acc_z+6, out.begin()); }
  void getMomentDes(std::vector<double>& out) { std::copy(moment_des, moment_des+6, out.begin()); }
  void getTCPForce(std::vector<double>& out) { std::copy(tcp_force, tcp_force+6, out.begin()); }
  void getTCPSpeed(std::vector<double>& out) { std::copy(tcp_speed, tcp_speed+6, out.begin()); }
  void getTCPWrench(std::vector<double>& out) { std::copy(tcp_wrench, tcp_wrench+6, out.begin()); }
  void getTCPPose(std::vector<double>& out) { std::copy(tcp_pose, tcp_pose+6, out.begin()); }
  double getTCPForceScalar() { return tcp_force_scalar; }
  double getTCPPower() { return tcp_power; }
  double getTCPPayload() { return tcp_payload; }
  double getPower() { return power; }

  //////////////////////////// Robot Mode States ////////////////////////////

  int getRobotModeID() { return robot_mode_id; }
  bool isPowerOnRobot() { return is_power_on_robot; }
  bool isSecurityStopped() { return is_security_stopped; }
  bool isEmergencyStopped() { return is_emergency_stopped; }
  bool isExtraButtonPressed() { return is_extra_button_pressed; }
  bool isPowerButtonPressed() { return is_power_button_pressed; }
  bool isSafetySignalSuchThatWeShouldStop() { return is_safety_signal_such_that_we_should_stop; }
  void getJointModeIDs(std::vector<int>& out) 
  { std::copy(joint_mode_ids, joint_mode_ids+6, out.begin()); }

  ///////////////////////////////// Data ////////////////////////////////////
  
  shared_int sequence;

  /////////////////////////// Robot Joint States ////////////////////////////
  shared_real q_act[6];
  shared_real qd_act[6];
  shared_real i_act[6];
  shared_real q_des[6];
  shared_real qd_des[6];
  shared_real qdd_des[6];
  shared_real i_des[6];
  shared_real acc_x[6];
  shared_real acc_y[6];
  shared_real acc_z[6];
  shared_real moment_des[6];
  shared_real tcp_force[6];
  shared_real tcp_speed[6];
  shared_real tcp_wrench[6];
  shared_real tcp_pose[6];
  shared_real tcp_force_scalar;
  shared_real tcp_power;
  shared_real tcp_payload;
  shared_real power;
  ///////////////////////////////////////////////////////////////////////////

  //////////////////////////// Robot Mode States ////////////////////////////
  shared_int robot_mode_id;
  shared_bool is_power_on_robot;
  shared_bool is_security_stopped;
  shared_bool is_emergency_stopped;
  shared_bool is_extra_button_pressed; /* The small black button on the back side of the tablet */
  shared_bool is_power_button_pressed;  /* The big green button on the tablet */
  shared_bool is_safety_signal_such_that_we_should_stop; /* This is from the safety stop interface */
  shared_int joint_mode_ids[6];
  ///////////////////////////////////////////////////////////////////////////
};
}


#endif // UR_STATE_DATA_H
