
#include <ur_ctrl_client/ur_robot_hw.h>

namespace ur
{

URRobotHW::URRobotHW(ros::NodeHandle& nh, std::vector<std::string>& joint_names) :
  nh_(nh)
{
  // Register individual joint interface handles
  for(int i=0;i<6;i++) {
    // Register joint states
    jnt_state_iface_.registerHandle(
        JointStateHandle(joint_names[i], &ur_state_data_.q_act[i], &ur_state_data_.qd_act[i], 
                                         &ur_state_data_.i_act[i]));

    // Register joint commands
    jnt_pos_vel_acc_iface_.registerHandle(
        PosVelAccJointHandle(jnt_state_iface_.getHandle(joint_names[i]), 
                             &pva_pos_cmd_[i], &pva_vel_cmd_[i], &pva_acc_cmd_[i]));
    jnt_vel_iface_.registerHandle(
        JointHandle(jnt_state_iface_.getHandle(joint_names[i]), &vel_cmd_[i]));
    jnt_torque_iface_.registerHandle(
        URTorqueJointHandle(jnt_state_iface_.getHandle(joint_names[i]), 
                            &torque_cmd_[i], &torque_vel_cmd_[i], &secu_torque_cmd_[i],
                            &softness_cmd_[i]));
  }
  // Register joint command mode
  jnt_mode_iface_.registerHandle(JointModeHandle("joint_command_mode", &joint_mode_));
  // Register configuration handle
  config_iface_.registerHandle(URConfigHandle("config_command", &ur_state_data_, &ur_config_cmd_,
                                              &config_lock_));

  // Register interfaces
  registerInterface(&jnt_state_iface_);
  registerInterface(&jnt_pos_vel_acc_iface_);
  registerInterface(&jnt_vel_iface_);
  registerInterface(&jnt_torque_iface_);
  registerInterface(&jnt_mode_iface_);
  registerInterface(&config_iface_);
}

void URRobotHW::init(std::string& robot_ip)
{
  ROS_INFO("Robot state connecting to IP address: %s", robot_ip.c_str());
  connection_.init(const_cast<char *>(robot_ip.c_str()), UR_COM_PORT);
  if(!connection_.isConnected())
    connection_.makeConnect();
  msg_man_.init(&connection_);
  ur_state_hdl_.init(&connection_, &ur_state_data_);
  msg_man_.add(&ur_state_hdl_);
  ur_joint_cmd_.sequence = 1;
  joint_mode_ = URJointCommandModes::EMPTY;
  clearCommands();
}

void URRobotHW::clearCommands()
{
  for(int i=0;i<6;i++) {
    // pos/vel/acc control
    pva_pos_cmd_[i] = NAN; 
    pva_vel_cmd_[i] = NAN; 
    pva_acc_cmd_[i] = NAN;

    // velocity only control
    vel_cmd_[i] = NAN; 

    // torque control
    torque_cmd_[i] = NAN;
    torque_vel_cmd_[i] = NAN; 
    secu_torque_cmd_[i] = NAN;
    softness_cmd_[i] = NAN;
  }
  ur_config_cmd_.clearCommands();
}

// This blocks until the state information message has been received 
// from simple_message connection.
void URRobotHW::read()
{
  ur_state_hdl_.reset();

  ros::Rate r(500);
  while(ros::ok()) {
    while(connection_.isReadyReceive(0) && ros::ok()) 
      msg_man_.spinOnce();
    if(ur_state_hdl_.hasUpdated())
      return; 
    r.sleep();
  }
}

void URRobotHW::write()
{

  // Process latest commands if they are updated
  ur_joint_cmd_.mode = joint_mode_;
  switch(ur_joint_cmd_.mode) {

    case URJointCommandModes::POS_VEL_ACC:
      for(int i=0;i<6;i++) {
        if(pva_pos_cmd_[i] != pva_pos_cmd_[i] || 
           pva_vel_cmd_[i] != pva_vel_cmd_[i] || 
           pva_acc_cmd_[i] != pva_acc_cmd_[i]) {
          // one of the commands is NAN, this mode is invalid
          ur_joint_cmd_.mode = URJointCommandModes::EMPTY;
          break;
        }
        ur_joint_cmd_.q[i] = pva_pos_cmd_[i]; 
        ur_joint_cmd_.qd[i] = pva_vel_cmd_[i]; 
        ur_joint_cmd_.qdd[i] = pva_acc_cmd_[i]; 
      }
      break;

    case URJointCommandModes::VEL:
      for(int i=0;i<6;i++) {
        if(vel_cmd_[i] != vel_cmd_[i]) {
          // one of the commands is NAN, this mode is invalid
          ur_joint_cmd_.mode = URJointCommandModes::EMPTY;
          break;
        }
        ur_joint_cmd_.qd[i] = vel_cmd_[i]; 
      }
      break;

    case URJointCommandModes::VEL_SEC_CTRL_TORQUE:
      for(int i=0;i<6;i++) {
        if(torque_cmd_[i] != torque_cmd_[i] || 
           torque_vel_cmd_[i] != torque_vel_cmd_[i] || 
           secu_torque_cmd_[i] != secu_torque_cmd_[i] || 
           softness_cmd_[i] != softness_cmd_[i]) {
          // one of the commands is NAN, this mode is invalid
          ur_joint_cmd_.mode = URJointCommandModes::EMPTY;
          break;
        }
        ur_joint_cmd_.control_torque[i] = torque_cmd_[i]; 
        ur_joint_cmd_.security_torque[i] = secu_torque_cmd_[i]; 
        ur_joint_cmd_.qd[i] = torque_vel_cmd_[i]; 
        ur_joint_cmd_.softness[i] = softness_cmd_[i]; 
      }
      break;

    case URJointCommandModes::EMPTY:
      break;

    default:
      ur_joint_cmd_.mode = URJointCommandModes::EMPTY;
      break;
  }

  // send joint commands
  ur_joint_cmd_.toSimpleMessage(ur::URMessageTypes::JOINT_CMD, ur_joint_cmd_msg_);
  connection_.sendMsg(ur_joint_cmd_msg_);
  ur_joint_cmd_.sequence++;

  ur_config_cmd_.toSimpleMessage(ur::URMessageTypes::CONFIG_CMD, ur_config_cmd_msg_);
  connection_.sendMsg(ur_config_cmd_msg_);
  clearCommands(); // set all the commands back to NAN
}

}
