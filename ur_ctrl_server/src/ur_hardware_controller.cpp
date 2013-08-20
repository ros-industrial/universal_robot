
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <ur_ctrl_server/ur_hardware_controller.h>
//#include <ur_ctrl_server/ur_util.h>

namespace ur {

URHardwareController::URHardwareController(SimpleSocket* socket_conn) 
  : URControllerInterface(socket_conn)
{
  for(int i=0;i<MSG_BUFFER_SIZE;i++)
    msg_buffer[i].text = msg_text_buffers[i];
}

int URHardwareController::initRobot(int argc, char** argv)
{
  double joint_vel[6];
  if(argc == 1) {
    for(int j=0;j<6;j++)
      joint_vel[j] = 0.1;
  } else {
    if(argc == 2) {
      for(int j=0;j<6;j++)
        joint_vel[j] = atof(argv[1]);
    } else {
      if(argc == 7) {
        for(int j=0;j<6;j++)
          joint_vel[j] = atof(argv[1+j]);
      } else {
        printf("Need joint velocities\n");
        return -1;
      }
    }
  }

  // Load UR configuration file
  printf("Loading robot configuration...\n");
  if(!configuration_load()) {
    printf("Loading configuration failed (.urcontrol dir should be in ~/).\n");
    exit(EXIT_FAILURE);
  }
  printf("success!");

#if 0
  // Make this thread RT priority
  struct sched_param sch_param;
  pid_t pid;
  printf("Setting RT priority\n");
  pid = getpid();
  sch_param.sched_priority = 99;
  if (sched_setscheduler(pid, SCHED_FIFO, &sch_param) == 0) {
    printf("- Priority set\n");
  } else {
    printf("- Priority not set, error: %i\n", errno);
    exit(EXIT_FAILURE);
  }
  ////////////////////////////

  ur::startRobot(); // go through startup procedure
#endif
  // ur::initializeJoints(joint_vel); // move joints joint_vel speed to initialize robot
  return 0;
}

void URHardwareController::readRobotState()
{
  // waits until the next control cycle (125Hz) and repopulates local variables
  robotinterface_read_state_blocking();

  // joint states
  robotinterface_get_actual(ur_state.q_act, ur_state.qd_act);
  robotinterface_get_actual_current(ur_state.i_act);
  robotinterface_get_tool_accelerometer_readings(ur_state.acc_x, ur_state.acc_y, ur_state.acc_z);
  ur_state.tcp_force_scalar = robotinterface_get_tcp_force_scalar();
  robotinterface_get_tcp_force(ur_state.tcp_force);
  robotinterface_get_tcp_speed(ur_state.tcp_speed);
  ur_state.tcp_power = robotinterface_get_tcp_power();
  ur_state.power = robotinterface_get_power();
  robotinterface_get_target(ur_state.q_des, ur_state.qd_des, ur_state.qdd_des);
  robotinterface_get_target_current(ur_state.i_des);
  robotinterface_get_target_moment(ur_state.moment_des);
  robotinterface_get_tcp_wrench(ur_state.tcp_wrench);
  robotinterface_get_tcp(ur_state.tcp_pose);
  ur_state.tcp_payload = robotinterface_get_tcp_payload();

  // config states
  ur_state.robot_mode_id = robotinterface_get_robot_mode();
  ur_state.is_power_on_robot = robotinterface_is_power_on_robot();
  ur_state.is_security_stopped = robotinterface_is_security_stopped();
  ur_state.is_emergency_stopped = robotinterface_is_emergency_stopped();
  ur_state.is_extra_button_pressed = robotinterface_is_extra_button_pressed(); 
  /* The button on the back side of the screen */
  ur_state.is_power_button_pressed = robotinterface_is_power_button_pressed();  
  /* The big green button on the controller box */
  ur_state.is_safety_signal_such_that_we_should_stop = 
    robotinterface_is_safety_signal_such_that_we_should_stop(); 
  /* This is from the safety stop interface */
  for(int i=0;i<6;i++)
    ur_state.joint_mode_ids[i] = robotinterface_get_joint_mode(i);

  // read error codes
  msg_count = robotinterface_get_message_count();
  for(int i=0;i<msg_count;i++) {
    robotinterface_get_message(&msg_buffer[i]);
  }
}

void URHardwareController::sendRobotCommands()
{
  ////////////////////////////// Joint commands ///////////////////////////////
  if(ur_state.robot_mode_id != ROBOT_RUNNING_MODE && ur_state.robot_mode_id != ROBOT_INITIALIZING_MODE) 
    // robot not running at the moment, don't bother commanding joints
    robotinterface_command_empty_command();

  else if(ur_state.sequence - latest_cmd_seq >= CMD_TIMEOUT)
    // we haven't received a new command for CMD_TIMEOUT cycles, stop moving
    robotinterface_command_empty_command();

  else if(jnt_cmd.mode == ur::URJointCommandModes::EMPTY)
    robotinterface_command_empty_command();

  else if(jnt_cmd.mode == ur::URJointCommandModes::VEL)
    robotinterface_command_velocity(jnt_cmd.qd);

  else if(jnt_cmd.mode == ur::URJointCommandModes::POS_VEL_ACC)
    robotinterface_command_position_velocity_acceleration(jnt_cmd.q, jnt_cmd.qd, jnt_cmd.qdd);

  else if(jnt_cmd.mode == ur::URJointCommandModes::VEL_SEC_CTRL_TORQUE)
    robotinterface_command_velocity_security_torque_control_torque(
        jnt_cmd.qd, jnt_cmd.security_torque, jnt_cmd.control_torque, jnt_cmd.softness);

  else if(jnt_cmd.mode == ur::URJointCommandModes::TORQUE)
    robotinterface_command_torque(jnt_cmd.qd, jnt_cmd.security_torque, jnt_cmd.control_torque);

  else
    robotinterface_command_empty_command();
  /////////////////////////////////////////////////////////////////////////////

  /////////////////////////////// Config commands ///////////////////////////////
  if(config_cmd.func_calls & URI_OPEN_REAL)
    open_result = robotinterface_open(0);

  else if(config_cmd.func_calls & URI_OPEN_SIMULATED)
    open_result = robotinterface_open(1);

  if(config_cmd.func_calls & URI_CLOSE)
    close_result = robotinterface_close();

  if(config_cmd.func_calls & URI_UNLOCK_SECURITY_STOP)
    unlock_security_stop_result = robotinterface_unlock_security_stop();

  if(config_cmd.func_calls & URI_SET_ROBOT_READY_MODE)
    robotinterface_set_robot_ready_mode();

  if(config_cmd.func_calls & URI_SET_ROBOT_RUNNING_MODE)
    robotinterface_set_robot_running_mode();

  if(config_cmd.func_calls & URI_POWER_ON_ROBOT)
    robotinterface_power_on_robot();

  if(config_cmd.func_calls & URI_POWER_OFF_ROBOT)
    robotinterface_power_off_robot();

  if(config_cmd.func_calls & URI_SET_TCP)
    robotinterface_set_tcp(config_cmd.set_tcp_pose);

  if(config_cmd.func_calls & URI_SET_TCP_PAYLOAD_COG)
    robotinterface_set_tcp_payload_cog(config_cmd.set_tcp_payload_cog);

  if(config_cmd.func_calls & URI_SET_TCP_PAYLOAD)
    robotinterface_set_tcp_payload(config_cmd.set_tcp_payload);

  if(config_cmd.func_calls & URI_SET_TCP_WRENCH)
    robotinterface_set_tcp_wrench(config_cmd.set_tcp_wrench, config_cmd.set_tcp_wrench_in_base_coord);
  
  if(config_cmd.func_calls & URI_SET_SECURITY_STOP)
    robotinterface_security_stop(config_cmd.security_stop_joint_code, 
                                 config_cmd.security_stop_error_state,
                                 config_cmd.security_stop_error_argument);
  /////////////////////////////////////////////////////////////////////////////

  // Send commands to lower level controller.
  // This must be called within a very short timeframe from when 
  // read_state_blocking was called.
  robotinterface_send();
}

}
