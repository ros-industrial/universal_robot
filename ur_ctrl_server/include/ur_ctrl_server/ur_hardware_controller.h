#ifndef UR_HARDWARE_CONTROLLER_H
#define UR_HARDWARE_CONTROLLER_H

#include <ur_ctrl_server/ur_controller_iface.h>
#include <robotinterface.h>
#include <Configuration.h>
#include <microprocessor_commands.h>
#include <microprocessor_definitions.h>

#define MSG_BUFFER_SIZE 10
#define CMD_TIMEOUT 3

namespace ur {

class URHardwareController : public URControllerInterface
{
protected:

  // Read state of the robot from UR robotinterface
  void readRobotState();
  // Send robot commands to joints using robotinterface
  void sendRobotCommands();

private:

  int open_result; // currently unused
  int close_result; // currently unused
  int unlock_security_stop_result; // currently unused

  ////////////////////////// Robot Error Messages ///////////////////////////
  struct message_t msg_buffer[MSG_BUFFER_SIZE];
  char msg_text_buffers[MSG_BUFFER_SIZE][100];
  int msg_count;
  ///////////////////////////////////////////////////////////////////////////

public:
  URHardwareController(SimpleSocket* socket_conn);
  ~URHardwareController() {}
  virtual int initRobot(int argc, char** argv);

};

typedef URHardwareController URController;

}


#endif // UR_HARDWARE_CONTROLLER_H
