
#include <ur_controllers/ur_state_controllers.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ur_controllers::StatePublishController,controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(ur_controllers::DiagnosticPublishController,controller_interface::ControllerBase)
