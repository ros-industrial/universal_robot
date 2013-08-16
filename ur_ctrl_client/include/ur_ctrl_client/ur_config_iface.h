/// \author Kelsey Hawkins

#ifndef UR_CONFIG_IFACE_H
#define UR_CONFIG_IFACE_H

#define FLOAT64

#include <cassert>
#include <string>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <ur_ctrl_server/simple_msgs/ur_config_cmd.h>
#include <ur_ctrl_server/simple_msgs/ur_state_data.h>

using hardware_interface::DontClaimResources;
using hardware_interface::HardwareInterfaceException;
using hardware_interface::HardwareInterface;
using hardware_interface::HardwareResourceManager;
typedef boost::recursive_mutex::scoped_lock scoped_lock;

namespace ur
{

/** \brief A handle used to read and command a joint mode. */
class URConfigHandle 
{
public:
  URConfigHandle() : name_(), state_data_(0), config_cmd_(0), config_lock_(0)  {}

  /**
   * \param name The name of the joint's mode handle. This should be unique per joint name,
   *             but shared across different modes.  Best to give it a name like
   *             "<joint name>_mode"
   * \param mode A pointer to the storage for this joint's command mode
   */
  URConfigHandle(const std::string& name, URStateData* state_data, URConfigCommand* config_cmd,
                 boost::recursive_mutex* config_lock) 
    : name_(name), state_data_(state_data), config_cmd_(config_cmd), config_lock_(config_lock)
  {
    if (!state_data_)
    {
      throw HardwareInterfaceException("Cannot create handle, state data pointer is null.");
    }
    if (!config_cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle, config command pointer is null.");
    }
  }

  std::string getName() const {return name_;}
  // URStateData* getStateData() const {assert(state_data_); return state_data_;}
  // URConfigCommand* getCommand() const {assert(config_cmd_); return config_cmd_;}

  /////////////////////////// Robot Joint States ////////////////////////////

  void getActualQ(std::vector<double>& out) { state_data_->getActualQ(out); }
  void getActualQD(std::vector<double>& out) { state_data_->getActualQD(out); }
  void getActualI(std::vector<double>& out) { state_data_->getActualI(out); }
  void getDesiredQ(std::vector<double>& out) { state_data_->getDesiredQ(out); }
  void getDesiredQD(std::vector<double>& out) { state_data_->getDesiredQD(out); }
  void getDesiredQDD(std::vector<double>& out) { state_data_->getDesiredQDD(out); }
  void getDesiredI(std::vector<double>& out) { state_data_->getDesiredI(out); }
  void getAccelX(std::vector<double>& out) { state_data_->getAccelX(out); }
  void getAccelY(std::vector<double>& out) { state_data_->getAccelY(out); }
  void getAccelZ(std::vector<double>& out) { state_data_->getAccelZ(out); }
  void getMomentDes(std::vector<double>& out) { state_data_->getMomentDes(out); }
  void getTCPForce(std::vector<double>& out) { state_data_->getTCPForce(out); }
  void getTCPSpeed(std::vector<double>& out) { state_data_->getTCPSpeed(out); }
  void getTCPWrench(std::vector<double>& out) { state_data_->getTCPWrench(out); }
  void getTCPPose(std::vector<double>& out) { state_data_->getTCPPose(out); }
  double getTCPForceScalar() { return state_data_->getTCPForceScalar(); }
  double getTCPPower() { return state_data_->getTCPPower(); }
  double getTCPPayload() { return state_data_->getTCPPayload(); }
  double getPower() { return state_data_->getPower(); }

  //////////////////////////// Robot Mode States ////////////////////////////

  int getRobotModeID() { return state_data_->getRobotModeID(); }
  bool isPowerOnRobot() { return state_data_->isPowerOnRobot(); }
  bool isSecurityStopped() { return state_data_->isSecurityStopped(); }
  bool isEmergencyStopped() { return state_data_->isEmergencyStopped(); }
  bool isExtraButtonPressed() { return state_data_->isExtraButtonPressed(); }
  bool isPowerButtonPressed() { return state_data_->isPowerButtonPressed(); }
  bool isSafetySignalSuchThatWeShouldStop() 
    { return state_data_->isSafetySignalSuchThatWeShouldStop(); }
  void getJointModeIDs(std::vector<int>& out) { state_data_->getJointModeIDs(out); }

  void addCommand(const URConfigCommand* new_cmd) 
    { scoped_lock guard(*config_lock_); config_cmd_->addCommand(new_cmd); }
  void clearCommands() 
    { scoped_lock guard(*config_lock_); config_cmd_->clearCommands(); }
  void openRealInterface() 
    { scoped_lock guard(*config_lock_); config_cmd_->openRealInterface(); }
  void openSimulatedInterface() 
    { scoped_lock guard(*config_lock_); config_cmd_->openSimulatedInterface(); }
  void closeInterface() 
    { scoped_lock guard(*config_lock_); config_cmd_->closeInterface(); }
  void unlockSecurityStop() 
    { scoped_lock guard(*config_lock_); config_cmd_->unlockSecurityStop(); }
  void setRobotReadyMode() 
    { scoped_lock guard(*config_lock_); config_cmd_->setRobotReadyMode(); }
  void setRobotRunningMode() 
    { scoped_lock guard(*config_lock_); config_cmd_->setRobotRunningMode(); }
  void powerOnRobot() 
    { scoped_lock guard(*config_lock_); config_cmd_->powerOnRobot(); }
  void powerOffRobot() 
    { scoped_lock guard(*config_lock_); config_cmd_->powerOffRobot(); }
  void setTCP(const std::vector<double>& tcp_pose) 
    { scoped_lock guard(*config_lock_); config_cmd_->setTCP(tcp_pose); }
  void setTCPPayloadCOG(const std::vector<double>& tcp_payload_cog) 
    { scoped_lock guard(*config_lock_); config_cmd_->setTCPPayloadCOG(tcp_payload_cog); }
  void setTCPPayload(double tcp_payload) 
    { scoped_lock guard(*config_lock_); config_cmd_->setTCPPayload(tcp_payload); }
  void setTCPWrenchBase(const std::vector<double>& tcp_wrench) 
    { scoped_lock guard(*config_lock_); config_cmd_->setTCPWrenchBase(tcp_wrench); }
  void setTCPWrenchEE(const std::vector<double>& tcp_wrench) 
    { scoped_lock guard(*config_lock_); config_cmd_->setTCPWrenchEE(tcp_wrench); }
  void setSecurityStop(char joint_code, int error_state, int error_argument) 
    { scoped_lock guard(*config_lock_); 
      config_cmd_->setSecurityStop(joint_code, error_state, error_argument); }

private:
  std::string name_;
  URStateData* state_data_;
  URConfigCommand* config_cmd_;
  boost::recursive_mutex* config_lock_;
};

/** \brief Hardware interface to support commanding a joint command config_cmd.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class URConfigInterface : 
  public HardwareResourceManager<URConfigHandle, DontClaimResources> {};

}

#endif
