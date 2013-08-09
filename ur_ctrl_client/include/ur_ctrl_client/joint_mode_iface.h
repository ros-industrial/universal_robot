/// \author Kelsey Hawkins

#ifndef JOINT_MODE_IFACE_H
#define JOINT_MODE_IFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a joint mode. */
class JointModeHandle 
{
public:
  JointModeHandle() : name_(), mode_(0) {}

  /**
   * \param name The name of the joint's mode handle. This should be unique per joint name,
   *             but shared across different modes.  Best to give it a name like
   *             "<joint name>_mode"
   * \param mode A pointer to the storage for this joint's command mode
   */
  JointModeHandle(const std::string& name, int* mode) : name_(name), mode_(mode)
  {
    if (!mode_)
    {
      throw HardwareInterfaceException("Cannot create handle, command mode pointer is null.");
    }
  }

  std::string getName() const {return name_;}
  void setMode(int mode) {assert(mode_); *mode_ = mode;}
  int getMode() const {assert(mode_); return *mode_;}

private:
  std::string name_;
  int* mode_;
};

/** \brief Hardware interface to support commanding a joint command mode.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class JointModeInterface : 
  public HardwareResourceManager<JointModeHandle, ClaimResources> {};

}

#endif
