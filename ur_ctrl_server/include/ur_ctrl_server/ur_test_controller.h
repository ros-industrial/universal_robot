#ifndef UR_TEST_CONTROLLER_H
#define UR_TEST_CONTROLLER_H

#include <ur_ctrl_server/ur_controller_iface.h>

#define CMD_TIMEOUT 3

namespace ur {

class URTestController : public URControllerInterface
{
protected:

  // Read state of the robot from UR robotinterface
  void readRobotState();
  // Send robot commands to joints using robotinterface
  void sendRobotCommands();

public:
  URTestController(SimpleSocket* socket_conn);
  ~URTestController() {}
  virtual int initRobot(int argc, char** argv);

};

typedef URTestController URController;
}

#endif // UR_TEST_CONTROLLER_H
