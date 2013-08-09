#ifndef UR_CTRL_SHARED_H
#define UR_CTRL_SHARED_H

#define UR_COM_PORT industrial::simple_socket::StandardSocketPorts::IO

namespace ur
{
namespace URMessageTypes
{
enum URMessageType
{
  STATE = 3000,
  JOINT_CMD = 3001,
  CONFIG_CMD = 3002
};
}
typedef URMessageTypes::URMessageType URMessageType;

namespace URJointCommandModes
{
enum URJointCommandMode
{
  EMPTY = 0,
  VEL = 1,
  POS_VEL_ACC = 2,
  VEL_SEC_CTRL_TORQUE = 3,
  TORQUE = 4
};
}
typedef URJointCommandModes::URJointCommandMode URJointCommandMode;

// Mode commands
#define URI_OPEN_REAL                (1 << 0)
#define URI_OPEN_SIMULATED           (1 << 1)
#define URI_CLOSE                    (1 << 2)
#define URI_UNLOCK_SECURITY_STOP     (1 << 3)
#define URI_SET_ROBOT_READY_MODE     (1 << 4)
#define URI_SET_ROBOT_RUNNING_MODE   (1 << 5) 
#define URI_POWER_ON_ROBOT           (1 << 6)
#define URI_POWER_OFF_ROBOT          (1 << 7)
#define URI_SET_TCP                  (1 << 8)
#define URI_SET_TCP_PAYLOAD_COG      (1 << 9)
#define URI_SET_TCP_PAYLOAD          (1 <<10)
#define URI_SET_TCP_WRENCH           (1 <<11)
#define URI_SET_SECURITY_STOP        (1 <<12)

}

#endif // UR_CTRL_SHARED_H
