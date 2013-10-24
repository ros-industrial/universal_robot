
#ifndef UR_FORWARD_CMD_CONTROLLERS_H
#define UR_FORWARD_CMD_CONTROLLERS_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>

#include <ur_ctrl_client/pos_vel_acc_joint_iface.h>
#include <ur_ctrl_client/ur_config_iface.h>

using hardware_interface::PosVelAccJointHandle;
using hardware_interface::PosVelAccJointInterface;
using hardware_interface::JointHandle;
using hardware_interface::VelocityJointInterface;

namespace ur_controllers
{

/**
 * \brief Forwards Position/Velocity/Acceleration array commands to the underlying controller.
 *
 * This class passes the command signals down to the set of joints specified.
 *
 * \section ROS interface
 *
 * \param joint_names Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (trajectory_msgs/JointTrajectoryPoint) : The joint commands to apply. The arrays must be the same length as the parameter joint_names.  time_from_start is unused.
 */
class PosVelAccForwardController: 
  public controller_interface::Controller<PosVelAccJointInterface>
{
public:
  PosVelAccForwardController() {}
  ~PosVelAccForwardController() {sub_command_.shutdown();}

  bool init(PosVelAccJointInterface* hw, ros::NodeHandle &n)
  {
    XmlRpc::XmlRpcValue v;
    n.param("joint_names", v, v);
    for(int i=0;i<v.size();i++)
      joints_.push_back(hw->getHandle(v[i]));
    pos_.resize(joints_.size());
    vel_.resize(joints_.size());
    acc_.resize(joints_.size());

    sub_command_ = n.subscribe<trajectory_msgs::JointTrajectoryPoint>("command", 1, 
                                        &PosVelAccForwardController::commandCB, this);
    return true;
  }

  void starting(const ros::Time& time) 
  { for(int i=0;i<joints_.size();i++) { pos_[i] = NAN; vel_[i] = NAN; acc_[i] = NAN; } }

  void update(const ros::Time& time, const ros::Duration& period) 
  {
    for(int i=0;i<joints_.size();i++) {
      joints_[i].setPosition(pos_[i]); 
      joints_[i].setVelocity(vel_[i]); 
      joints_[i].setAcceleration(acc_[i]);
      pos_[i] = NAN; vel_[i] = NAN; acc_[i] = NAN;
    }
  }

private:
  std::vector<hardware_interface::PosVelAccJointHandle> joints_;
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> acc_;

  ros::Subscriber sub_command_;
  void commandCB(const trajectory_msgs::JointTrajectoryPointConstPtr& msg) 
  {
    if(msg->positions.size() != pos_.size()) {
      ROS_ERROR("PosVelAccForwardController: positions size %d not of same length as joint_names %d",
                msg->positions.size(), pos_.size());
      return;
    }
    if(msg->velocities.size() != vel_.size()) {
      ROS_ERROR("PosVelAccForwardController: velocities size %d not of same length as joint_names %d",
                msg->velocities.size(), vel_.size());
      return;
    }
    if(msg->accelerations.size() != acc_.size()) {
      ROS_ERROR("PosVelAccForwardController: accelerations size %d not of same length as joint_names %d",
                msg->accelerations.size(), acc_.size());
      return;
    }
    for(int i=0;i<joints_.size();i++) {
      pos_[i] = msg->positions[i];
      vel_[i] = msg->velocities[i];
      acc_[i] = msg->accelerations[i];
    }
  }
};

/**
 * \brief Forwards velocity array commands to the underlying controller.
 *
 * This class passes the command signals down to the set of joints specified.
 *
 * \section ROS interface
 *
 * \param joint_names Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs/MultiFloat64Array) : The joint commands to apply. The arrays must be the same length as the parameter joint_names.  time_from_start is unused.
 */
class VelocityForwardController: 
  public controller_interface::Controller<VelocityJointInterface>
{
public:
  VelocityForwardController() {}
  ~VelocityForwardController() {sub_command_.shutdown();}

  bool init(VelocityJointInterface* hw, ros::NodeHandle &n)
  {
    if(!n.hasParam("joint_names")) {
      ROS_ERROR("VelocityForwardController: parameter joint_names must be defined");
      return false;
    }
    XmlRpc::XmlRpcValue v;
    n.param("joint_names", v, v);
    for(int i=0;i<v.size();i++)
      joints_.push_back(hw->getHandle(v[i]));
    vel_.resize(joints_.size());

    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, 
                                        &VelocityForwardController::commandCB, this);
    return true;
  }

  void starting(const ros::Time& time) 
  { for(int i=0;i<joints_.size();i++) { vel_[i] = NAN; } }

  void update(const ros::Time& time, const ros::Duration& period) 
  {
    for(int i=0;i<joints_.size();i++) {
      joints_[i].setCommand(vel_[i]); 
      vel_[i] = NAN;
    }
  }

private:
  std::vector<hardware_interface::JointHandle> joints_;
  std::vector<double> vel_;

  ros::Subscriber sub_command_;
  void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg) 
  {
    if(msg->data.size() != vel_.size()) {
      ROS_WARN("VelocityForwardController: velocities size %d not of same length as joint_names %d",
               msg->data.size(), vel_.size());
      return;
    }
    for(int i=0;i<joints_.size();i++) {
      vel_[i] = msg->data[i];
    }
  }
};

/**
 * \brief Forwards velocity array commands to the underlying controller.
 *
 * This class passes the command signals down to the set of joints specified.
 *
 * \section ROS interface
 *
 * \param joint_names Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs/MultiFloat64Array) : The joint commands to apply. The arrays must be the same length as the parameter joint_names.  time_from_start is unused.
 */
class ConfigForwardController: 
  public controller_interface::Controller<ur::URConfigInterface>
{
public:
  ConfigForwardController() {}
  ~ConfigForwardController() {for(size_t i=0;i<sub_list_.size();i++) sub_list_[i].shutdown();}

  bool init(ur::URConfigInterface* hw, ros::NodeHandle &n)
  {
    config_hdl_ = hw->getHandle("config_command");

    sub_list_.push_back(n.subscribe<std_msgs::Empty>("open_interface", 1, 
                                        &ConfigForwardController::openRealInterfaceCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Empty>("open_simulated_interface", 1, 
                                        &ConfigForwardController::openSimulatedInterfaceCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Empty>("close_interface", 1, 
                                        &ConfigForwardController::closeInterfaceCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Empty>("unlock_security_stop", 1, 
                                        &ConfigForwardController::unlockSecurityStopCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Empty>("set_robot_ready_mode", 1, 
                                        &ConfigForwardController::setRobotReadyModeCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Empty>("set_robot_running_mode", 1, 
                                        &ConfigForwardController::setRobotRunningModeCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Empty>("set_freedrive_mode", 1, 
                                        &ConfigForwardController::setFreedriveModeCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Empty>("power_on_robot", 1, 
                                        &ConfigForwardController::powerOnRobotCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Empty>("power_off_robot", 1, 
                                        &ConfigForwardController::powerOffRobotCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Float64MultiArray>("set_tcp", 1, 
                                        &ConfigForwardController::setTCPCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Float64MultiArray>("set_tcp_payload_cog", 1, 
                                        &ConfigForwardController::setTCPPayloadCOGCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Float64>("set_tcp_payload", 1, 
                                        &ConfigForwardController::setTCPPayloadCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Float64MultiArray>("set_tcp_wrench_ee", 1, 
                                        &ConfigForwardController::setTCPWrenchEECB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Float64MultiArray>("set_tcp_wrench_base", 1, 
                                        &ConfigForwardController::setTCPWrenchBaseCB, this));
    sub_list_.push_back(n.subscribe<std_msgs::Int32MultiArray>("set_security_stop", 1, 
                                        &ConfigForwardController::setSecurityStopCB, this));

    config_cmd_buf_.resize(sub_list_.size());
    for(size_t i=0;i<config_cmd_buf_.size();i++) {
      config_cmd_buf_[i].clearCommands();
    }
    return true;
  }

  void starting(const ros::Time& time) 
  { 
    for(size_t i=0;i<config_cmd_buf_.size();i++) {
      config_cmd_buf_[i].clearCommands();
    }
  }

  void update(const ros::Time& time, const ros::Duration& period) 
  {
    // I think we need to be careful here since commands can come in during this loop
    config_hdl_.clearCommands();
    for(size_t i=0;i<config_cmd_buf_.size();i++) {
      config_hdl_.addCommand(&config_cmd_buf_[i]);
      config_cmd_buf_[i].clearCommands();
    }
  }

private:
  ur::URConfigHandle config_hdl_;
  std::vector<ur::URConfigCommand> config_cmd_buf_;

  std::vector<ros::Subscriber> sub_list_;
  void openRealInterfaceCB(const std_msgs::EmptyConstPtr& msg) 
  { config_cmd_buf_[0].openRealInterface(); }
  void openSimulatedInterfaceCB(const std_msgs::EmptyConstPtr& msg)
  { config_cmd_buf_[1].openSimulatedInterface(); }
  void closeInterfaceCB(const std_msgs::EmptyConstPtr& msg)
  { config_cmd_buf_[2].closeInterface(); }
  void unlockSecurityStopCB(const std_msgs::EmptyConstPtr& msg)
  { config_cmd_buf_[3].unlockSecurityStop(); }
  void setRobotReadyModeCB(const std_msgs::EmptyConstPtr& msg)
  { config_cmd_buf_[4].setRobotReadyMode(); }
  void setRobotRunningModeCB(const std_msgs::EmptyConstPtr& msg)
  { config_cmd_buf_[5].setRobotRunningMode(); }
  void powerOnRobotCB(const std_msgs::EmptyConstPtr& msg)
  { config_cmd_buf_[6].powerOnRobot(); }
  void powerOffRobotCB(const std_msgs::EmptyConstPtr& msg)
  { config_cmd_buf_[7].powerOffRobot(); }
  void setTCPCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  { config_cmd_buf_[8].setTCP(msg->data); }
  void setTCPPayloadCOGCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  { config_cmd_buf_[9].setTCPPayloadCOG(msg->data); }
  void setTCPPayloadCB(const std_msgs::Float64ConstPtr& msg)
  { config_cmd_buf_[10].setTCPPayload(msg->data); }
  void setTCPWrenchBaseCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  { config_cmd_buf_[11].setTCPWrenchBase(msg->data); }
  void setTCPWrenchEECB(const std_msgs::Float64MultiArrayConstPtr& msg)
  { config_cmd_buf_[12].setTCPWrenchEE(msg->data); }
  void setFreedriveModeCB(const std_msgs::EmptyConstPtr& msg)
  { config_cmd_buf_[13].setFreedriveMode(); }
  void setSecurityStopCB(const std_msgs::Int32MultiArrayConstPtr& msg)
  { 
    if(msg->data.size() != 3) {
      ROS_WARN("ConfigForwardController: Security stop must be of length 3");
      return;
    }
    config_cmd_buf_[14].setSecurityStop(msg->data[0], msg->data[1], msg->data[2]);
  }
};

}

#endif
