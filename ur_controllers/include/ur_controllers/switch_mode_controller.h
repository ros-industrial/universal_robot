
/// \author Kelsey Hawkins
//
#ifndef SWITCH_MODE_CONTROLLER_H
#define SWITCH_MODE_CONTROLLER_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Int32.h>
#include <ur_ctrl_client/joint_mode_iface.h>

using hardware_interface::JointModeHandle;
using hardware_interface::JointModeInterface;

namespace switch_mode_controller
{

/**
 * \brief Single joint controller.
 *
 * This class passes the command signal down to the joint.
 * Command signal and joint hardware interface are of the same type, e.g. effort commands for an effort-controlled
 * joint.
 *
 * \tparam T Type implementing the JointCommandInterface.
 *
 * \section ROS interface
 *
 * \param type hardware interface type.
 * \param joint Name of the joint to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64) : The joint command to apply.
 */
class SwitchModeController: 
  public controller_interface::Controller<JointModeInterface>
{
public:
  SwitchModeController() {}
  ~SwitchModeController() {}

  bool init(JointModeInterface* hw, ros::NodeHandle &n)
  {
    XmlRpc::XmlRpcValue v;
    n.param("joint_names", v, v);
    std::string suffix("_mode");
    n.param("joint_mode_suffix", suffix, suffix);
    for(int i=0;i<v.size();i++)
      joints_.push_back(hw->getHandle(std::string(v[i]) + suffix));

    return true;
  }

  void starting(const ros::Time& time) 
  {
    for(size_t i=0;i<joints_.size();i++)
      joints_[i].setMode(getMode());
  }

  void stopping(const ros::Time& time) 
  {
    for(size_t i=0;i<joints_.size();i++)
      joints_[i].setMode(0);
  }

  void update(const ros::Time& time, const ros::Duration& period) {}

  virtual int getMode() const = 0;

private:
  std::vector<JointModeHandle> joints_;

};

}

#endif
