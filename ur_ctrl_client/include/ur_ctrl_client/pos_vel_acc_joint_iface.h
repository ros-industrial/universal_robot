/// \author Kelsey Hawkins

#ifndef POS_VEL_ACC_JOINT_CMD_INTERFACE_H
#define POS_VEL_ACC_JOINT_CMD_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class PosVelAccJointHandle : public JointHandle
{
public:
  PosVelAccJointHandle() : hardware_interface::JointHandle(), vel_(0), acc_(0) {}

  /**
   * \param js This joint's handle
   * \param pos A pointer to the storage for this joint's position command
   * \param vel A pointer to the storage for this joint's velocity command
   * \param acc A pointer to the storage for this joint's acceleration command
   */
  PosVelAccJointHandle(const JointStateHandle& js, double* pos, double* vel, double* acc)
    : JointHandle(js, pos), vel_(vel), acc_(acc)
  {
    if (!vel_ || !acc_)
    {
      throw HardwareInterfaceException(
          "Cannot create handle '" + js.getName() + "'. Some command data pointer is null.");
    }
  }

  void   setPosition(double pos) {setCommand(pos);}
  double getPosition() const {return getCommand();}

  void   setVelocity(double vel) {assert(vel_); *vel_ = vel;}
  double getVelocity() const {assert(vel_); return *vel_;}

  void   setAcceleration(double acc) {assert(acc_); *acc_ = acc;}
  double getAcceleration() const {assert(acc_); return *acc_;}

private:
  double* vel_;
  double* acc_;
};

/** \brief Hardware interface to support commanding an array of joints by position velocity and acceleration.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class PosVelAccJointInterface : 
  public HardwareResourceManager<PosVelAccJointHandle, ClaimResources> {};

}

#endif
