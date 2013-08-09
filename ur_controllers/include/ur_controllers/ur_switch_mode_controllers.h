
#include <ur_controllers/switch_mode_controller.h>
#include "ur_ctrl_server/ur_ctrl_shared.h"

namespace ur_controllers
{

class PosVelAccModeController : public switch_mode_controller::SwitchModeController
{
  virtual int getMode() const { return ur::URJointCommandModes::POS_VEL_ACC; }
};

class VelocityModeController : public switch_mode_controller::SwitchModeController
{
  virtual int getMode() const { return ur::URJointCommandModes::VEL; }
};

}
