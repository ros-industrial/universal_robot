#ifndef UR_CONTROLLER_IFACE_H
#define UR_CONTROLLER_IFACE_H

#define LINUXSOCKETS
#define FLOAT64

#include "simple_message/socket/simple_socket.h"
#include "simple_message/message_manager.h"

#include "ur_ctrl_server/ur_joint_cmd_handler.h"
#include "ur_ctrl_server/ur_config_cmd_handler.h"
#include "ur_ctrl_server/ur_ctrl_shared.h"
#include "ur_ctrl_server/simple_msgs/ur_state_data.h"

using industrial::simple_socket::SimpleSocket;
using industrial::message_manager::MessageManager;
using industrial::simple_message::SimpleMessage;

namespace ur {

class URControllerInterface
{
protected:
  int latest_cmd_seq;
  //bool was_emergency_stopped;

  /////////////////////// simple_message Communication //////////////////////
private:
  SimpleSocket* connection;
  MessageManager msg_man;
  URJointCommandHandler joint_cmd_handler;
  URConfigCommandHandler config_cmd_handler;

protected:
  URJointCommand jnt_cmd;
  URConfigCommand config_cmd;
  SimpleMessage ur_state_simp_msg;
  URStateData ur_state;
  ///////////////////////////////////////////////////////////////////////////

  // Read state of the robot from microcontrollers
  virtual void readRobotState() = 0;
  // Send robot commands to joint microcontrollers
  virtual void sendRobotCommands() = 0;

private:
  // Send state and receive commands through simple_message to ROS client
  void sendAndReceiveMessages();

public:
  URControllerInterface(SimpleSocket* socket_conn);
  ~URControllerInterface();

  virtual int initRobot(int argc, char** argv) {}

  void controlLoop();
};

}
#endif // UR_CONTROLLER_IFACE_H
