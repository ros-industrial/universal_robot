
#include <ur_controllers/ur_switch_mode_controllers.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ur_controllers::PosVelAccModeController,controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(ur_controllers::VelocityModeController,controller_interface::ControllerBase)
