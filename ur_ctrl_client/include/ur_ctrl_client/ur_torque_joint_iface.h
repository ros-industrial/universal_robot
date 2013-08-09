/// \author Kelsey Hawkins

#ifndef UR_TORQUE_JOINT_CMD_INTERFACE_H
#define UR_TORQUE_JOINT_CMD_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

using hardware_interface::JointHandle;
using hardware_interface::ClaimResources;
using hardware_interface::HardwareInterfaceException;
using hardware_interface::HardwareInterface;
using hardware_interface::HardwareResourceManager;

namespace ur
{

/** \brief A handle used to read and command a single joint. */
class URTorqueJointHandle : public JointHandle
{
public:
  URTorqueJointHandle() : hardware_interface::JointHandle(), 
                          vel_(0), security_torque_(0), softness_(0) {}

  /**
   * \param js This joint's handle
   * \param command_torque A pointer to the storage for this joint's torque command
   * \param vel A pointer to the storage for this joint's velocity command
   * \param security_torque A pointer to the storage for this joint's security/expected torque command
   * \param softness A pointer to the storage for this joint's softness/velocity gains command
   */
  URTorqueJointHandle(const JointStateHandle& js, 
                      double* command_torque, double* vel, double* security_torque, double* softness)
    : JointHandle(js, command_torque), 
      vel_(vel), security_torque_(security_torque_), softness_(softness)
  {
    if (!vel_ || !security_torque_ || !softness_)
    {
      throw HardwareInterfaceException(
          "Cannot create handle '" + js.getName() + "'. Some command data pointer is null.");
    }
  }

  void   setControlTorque(double torque) {setCommand(torque);}
  double getControlTorque() const {return getCommand();}

  void   setSecurityTorque(double security_torque) 
  {
    assert(security_torque_); 
    *security_torque_ = security_torque;
  }
  double getSecurityTorque() const                 
  {
    assert(security_torque_); 
    return *security_torque_;
  }

  void   setVelocity(double vel) {assert(vel_); *vel_ = vel;}
  double getVelocity() const {assert(vel_); return *vel_;}

  void   setSoftness(double softness) {assert(softness_); *softness_ = softness;}
  double getSoftness() const {assert(softness_); return *softness_;}

private:
  double* vel_;
  double* security_torque_;
  double* softness_;
};

/** \brief Hardware interface to support commanding an array of joints by Universal's torque specs.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class URTorqueJointInterface : 
  public HardwareResourceManager<URTorqueJointHandle, ClaimResources> {};

}

#endif
