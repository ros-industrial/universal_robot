#ifndef UR_STATE_CONTROLLERS_H
#define UR_STATE_CONTROLLERS_H

#define FLOAT64

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

#include <ur_ctrl_client/ur_config_iface.h>

using realtime_tools::RealtimePublisher;
using boost::lexical_cast;

namespace ur_controllers
{

class StatePublishController: 
  public controller_interface::Controller<ur::URConfigInterface>
{
public:
  StatePublishController() {}

  bool init(ur::URConfigInterface* hw, ros::NodeHandle &n)
  {
    config_hdl_ = hw->getHandle("config_command");

    mode_id_rt_pub_.reset(new RealtimePublisher<std_msgs::Int32>(
                          n, "robot_mode_id", 1, true));
    jnt_modes_rt_pub_.reset(new RealtimePublisher<std_msgs::Int32MultiArray>(
                            n, "joint_mode_ids", 1, true));
    jnt_modes_rt_pub_->msg_.data.resize(6);

    bool_rt_pubs_.resize(6);
    bool_rt_pubs_[0].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_power_on_robot", 1, true));
    bool_rt_pubs_[1].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_security_stopped", 1, true));
    bool_rt_pubs_[2].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_emergency_stopped", 1, true));
    bool_rt_pubs_[3].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_extra_button_pressed", 1, true));
    bool_rt_pubs_[4].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_power_button_pressed", 1, true));
    bool_rt_pubs_[5].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_safety_signal_such_that_we_should_stop", 1, true));
    cur_bool_states_.resize(bool_rt_pubs_.size());

    cur_joint_ids_.resize(6);

    return true;
  }

  void starting(const ros::Time& time) 
  { 
    first_update_ = true;
  }

  void update(const ros::Time& time, const ros::Duration& period) 
  {
    cur_bool_states_[0] = config_hdl_.isPowerOnRobot();
    cur_bool_states_[1] = config_hdl_.isSecurityStopped();
    cur_bool_states_[2] = config_hdl_.isEmergencyStopped();
    cur_bool_states_[3] = config_hdl_.isExtraButtonPressed();
    cur_bool_states_[4] = config_hdl_.isPowerButtonPressed();
    cur_bool_states_[5] = config_hdl_.isSafetySignalSuchThatWeShouldStop();

    // try to publish
    if(mode_id_rt_pub_->trylock()) {
      if(mode_id_rt_pub_->msg_.data != config_hdl_.getRobotModeID() || first_update_) {
        mode_id_rt_pub_->msg_.data = config_hdl_.getRobotModeID();
        mode_id_rt_pub_->unlockAndPublish();
      }
      else mode_id_rt_pub_->unlock();
    }

    if(jnt_modes_rt_pub_->trylock()) {
      config_hdl_.getJointModeIDs(cur_joint_ids_);
      bool joint_ids_changed = false;
      for(int i=0;i<6;i++) {
        if(jnt_modes_rt_pub_->msg_.data[i] != cur_joint_ids_[i]) {
          joint_ids_changed = true;
          break;
        }
      }
      if(joint_ids_changed || first_update_) {
        std::copy(cur_joint_ids_.begin(), cur_joint_ids_.end(), jnt_modes_rt_pub_->msg_.data.begin());
        jnt_modes_rt_pub_->unlockAndPublish();
      }
      else jnt_modes_rt_pub_->unlock();
    }

    for(size_t i=0;i<cur_bool_states_.size();i++) {
      if(bool_rt_pubs_[i]->trylock()) {
        if(bool_rt_pubs_[i]->msg_.data != cur_bool_states_[i] || first_update_) {
          bool_rt_pubs_[i]->msg_.data = cur_bool_states_[i];
          bool_rt_pubs_[i]->unlockAndPublish();
        }
        else bool_rt_pubs_[i]->unlock();
      }
    }
    first_update_ = false;
  }

  void stopping(const ros::Time& time) 
  {
  }

private:
  ur::URConfigHandle config_hdl_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Int32> > mode_id_rt_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Int32MultiArray> > jnt_modes_rt_pub_;
  std::vector<boost::shared_ptr<RealtimePublisher<std_msgs::Bool> > > bool_rt_pubs_;
  bool first_update_;
  std::vector<bool> cur_bool_states_;
  std::vector<int> cur_joint_ids_;
};

const char* JOINT_NAMES[6] = { 
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint"};

const char* ROBOT_MODES[11] = {
  "RUNNING",
  "FREEDRIVE",
  "READY",
  "INITIALIZING",
  "SECURITY_STOPPED",
  "EMERGENCY_STOPPED",
  "FATAL_ERROR",
  "NO_POWER",
  "NOT_CONNECTED",
  "SHUTDOWN",
  "SAFEGUARD_STOP"};

const char* JOINT_MODES[19] = {
  "PART_D_CALIBRATION",
  "BACKDRIVE",
  "POWER_OFF",
  "EMERGENCY_STOPPED",
  "CALVAL_INITIALIZATION",
  "ERROR",
  "FREEDRIVE",
  "SIMULATED 244",
  "NOT_RESPONDING",
  "MOTOR_INITIALISATION",
  "BOOTING",
  "PART_D_CALIBRATION_ERROR",
  "BOOTLOADER",
  "CALIBRATION",
  "SECURITY_STOPPED",
  "FAULT",
  "RUNNING",
  "INITIALISATION",
  "IDLE"};

class DiagnosticPublishController: 
  public controller_interface::Controller<ur::URConfigInterface>
{
public:
  DiagnosticPublishController() {}

  bool init(ur::URConfigInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    config_hdl_ = hw->getHandle("config_command");

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    diag_pub_.reset(new RealtimePublisher<diagnostic_msgs::DiagnosticArray>(
                     root_nh, "/diagnostics", 1));
    diag_pub_->msg_.status.resize(7);

    diag_pub_->msg_.status[0].name = "ur_arm: UR arm controller";
    diag_pub_->msg_.status[0].values.resize(8);
    diag_pub_->msg_.status[0].values[0].key = "Robot mode";
    diag_pub_->msg_.status[0].values[1].key = "Robot mode ID";
    diag_pub_->msg_.status[0].values[2].key = "Power on robot";
    diag_pub_->msg_.status[0].values[3].key = "Security stopped";
    diag_pub_->msg_.status[0].values[4].key = "Emergency stopped";
    diag_pub_->msg_.status[0].values[5].key = "Extra button pressed";
    diag_pub_->msg_.status[0].values[6].key = "Power button pressed";
    diag_pub_->msg_.status[0].values[7].key = "Safety signal such that we should stop";

    for(int i=0;i<6;i++) {
      diag_pub_->msg_.status[i+1].name = "ur_arm/" + std::string(JOINT_NAMES[i]) + ": UR arm joint";
      diag_pub_->msg_.status[i+1].values.resize(2);
      diag_pub_->msg_.status[i+1].values[0].key = "Joint mode";
      diag_pub_->msg_.status[i+1].values[1].key = "Joint mode ID";
    }
    
    cur_bool_states_.resize(6);
    cur_joint_ids_.resize(6);
    last_bool_states_.resize(6);
    last_joint_ids_.resize(6);

    return true;
  }

  void starting(const ros::Time& time) 
  { 
    if(publish_rate_ > 0.0)
      last_publish_time_ = time - ros::Duration(2.0/publish_rate_);
  }

  // only update at a rate of publish_rate_, or when the topics change
  void update(const ros::Time& time, const ros::Duration& period) 
  {
    bool update_diag = false;
    // limit rate of publishing
    if(publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time) {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
      update_diag = true;
    }

    cur_robot_mode_ = config_hdl_.getRobotModeID();
    cur_bool_states_[0] = config_hdl_.isPowerOnRobot();
    cur_bool_states_[1] = config_hdl_.isSecurityStopped();
    cur_bool_states_[2] = config_hdl_.isEmergencyStopped();
    cur_bool_states_[3] = config_hdl_.isExtraButtonPressed();
    cur_bool_states_[4] = config_hdl_.isPowerButtonPressed();
    cur_bool_states_[5] = config_hdl_.isSafetySignalSuchThatWeShouldStop();
    config_hdl_.getJointModeIDs(cur_joint_ids_);

    update_diag = update_diag || cur_robot_mode_ != last_robot_mode_;
    for(size_t i=0;i<cur_bool_states_.size();i++) 
      update_diag = update_diag || cur_bool_states_[i] != last_bool_states_[i];
    for(int i=0;i<6;i++)
      update_diag = update_diag || cur_joint_ids_[i] != last_joint_ids_[i];

    if(!update_diag) return;

    last_robot_mode_ = cur_robot_mode_;
    std::copy(cur_bool_states_.begin(), cur_bool_states_.end(), last_bool_states_.begin());
    std::copy(cur_joint_ids_.begin(), cur_joint_ids_.end(), last_joint_ids_.begin());

    if(diag_pub_->trylock()) {
      diag_pub_->msg_.header.stamp = time;

      // Robot modes
      if(cur_robot_mode_ >= 0 && cur_robot_mode_ < 2)
        diag_pub_->msg_.status[0].level = diag_pub_->msg_.status[0].OK;
      else if(cur_robot_mode_ >= 2 && cur_robot_mode_ < 4)
        diag_pub_->msg_.status[0].level = diag_pub_->msg_.status[0].WARN;
      else 
        diag_pub_->msg_.status[0].level = diag_pub_->msg_.status[0].ERROR;
      if(cur_robot_mode_ >= 0 && cur_robot_mode_ < 11)
        diag_pub_->msg_.status[0].message = ROBOT_MODES[cur_robot_mode_];
      else
        diag_pub_->msg_.status[0].message = "UNINITIALIZED";

      diag_pub_->msg_.status[0].values[0].value = diag_pub_->msg_.status[0].message;
      for(size_t i=0;i<cur_bool_states_.size();i++) 
        diag_pub_->msg_.status[0].values[i+1].value = lexical_cast<std::string>(cur_bool_states_[i]);
      
      // Joint modes
      for(int i=0;i<6;i++) {
        if(cur_joint_ids_[i] == 16)
          diag_pub_->msg_.status[i+1].level = diag_pub_->msg_.status[i+1].OK;
        else if(cur_joint_ids_[i] == 13 ||cur_joint_ids_[i] == 17 || cur_joint_ids_[i] == 18)
          diag_pub_->msg_.status[i+1].level = diag_pub_->msg_.status[i+1].WARN;
        else 
          diag_pub_->msg_.status[i+1].level = diag_pub_->msg_.status[i+1].ERROR;
        if(cur_joint_ids_[i] >= 0 && cur_joint_ids_[i] < 19)
          diag_pub_->msg_.status[i+1].message = JOINT_MODES[cur_joint_ids_[i]];
        else
          diag_pub_->msg_.status[i+1].message = "UNINITIALIZED";
        diag_pub_->msg_.status[i+1].values[0].value = diag_pub_->msg_.status[i+1].message;
        diag_pub_->msg_.status[i+1].values[1].value = lexical_cast<std::string>(cur_joint_ids_[i]);
      }

      diag_pub_->unlockAndPublish();
    }
  }

  void stopping(const ros::Time& time) 
  {
  }

private:

  ur::URConfigHandle config_hdl_;
  boost::shared_ptr<RealtimePublisher<diagnostic_msgs::DiagnosticArray> > diag_pub_;

  int cur_robot_mode_;
  std::vector<bool> cur_bool_states_;
  std::vector<int> cur_joint_ids_;

  int last_robot_mode_;
  std::vector<bool> last_bool_states_;
  std::vector<int> last_joint_ids_;

  ros::Time last_publish_time_;
  double publish_rate_;
};



}

#endif
