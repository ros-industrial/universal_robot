
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <ur_ctrl_server/ur_test_controller.h>

namespace ur {

URTestController::URTestController(SimpleSocket* socket_conn) 
  : URControllerInterface(socket_conn)
{
}

int URTestController::initRobot(int argc, char** argv)
{
  ur_state.robot_mode_id = 8;
  return 0;
}

void URTestController::readRobotState()
{
  for(int i=0;i<6;i++)
    ur_state.joint_mode_ids[i] = 16;

  usleep(1000000); // 1000000 us = 1 Hz
  // usleep(8000); // 8000 us = 125 Hz
}

void URTestController::sendRobotCommands()
{
  ////////////////////////////// Joint commands ///////////////////////////////

  if(jnt_cmd.mode == ur::URJointCommandModes::EMPTY) {
    printf("Empty command\n");
  }

  else if(jnt_cmd.mode == ur::URJointCommandModes::VEL) {
    printf("Velocity command: %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.qd[0], jnt_cmd.qd[1], jnt_cmd.qd[2], jnt_cmd.qd[3], jnt_cmd.qd[4], jnt_cmd.qd[5]);
  }

  else if(jnt_cmd.mode == ur::URJointCommandModes::POS_VEL_ACC) {
    printf("PosVelAcc command (pos): %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.q[0], jnt_cmd.q[1], jnt_cmd.q[2], jnt_cmd.q[3], jnt_cmd.q[4], jnt_cmd.q[5]);
    printf("                  (vel): %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.qd[0], jnt_cmd.qd[1], jnt_cmd.qd[2], jnt_cmd.qd[3], jnt_cmd.qd[4], jnt_cmd.qd[5]);
    printf("                  (acc): %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.qdd[0], jnt_cmd.qdd[1], jnt_cmd.qdd[2], jnt_cmd.qdd[3], jnt_cmd.qdd[4], jnt_cmd.qdd[5]);
  }

  else if(jnt_cmd.mode == ur::URJointCommandModes::VEL_SEC_CTRL_TORQUE) {
    printf("VSCTorque command (vel): %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.qd[0], jnt_cmd.qd[1], jnt_cmd.qd[2], jnt_cmd.qd[3], jnt_cmd.qd[4], jnt_cmd.qd[5]);
    printf("                  (sec): %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.security_torque[0], jnt_cmd.security_torque[1], jnt_cmd.security_torque[2], 
        jnt_cmd.security_torque[3], jnt_cmd.security_torque[4], jnt_cmd.security_torque[5]);
    printf("                  (tor): %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.control_torque[0], jnt_cmd.control_torque[1], jnt_cmd.control_torque[2], 
        jnt_cmd.control_torque[3], jnt_cmd.control_torque[4], jnt_cmd.control_torque[5]);
    printf("                  (sof): %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.softness[0], jnt_cmd.softness[1], jnt_cmd.softness[2], 
        jnt_cmd.softness[3], jnt_cmd.softness[4], jnt_cmd.softness[5]);
  }

  else if(jnt_cmd.mode == ur::URJointCommandModes::TORQUE) {
    printf("Torque command (vel): %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.qd[0], jnt_cmd.qd[1], jnt_cmd.qd[2], jnt_cmd.qd[3], jnt_cmd.qd[4], jnt_cmd.qd[5]);
    printf("               (sec): %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.security_torque[0], jnt_cmd.security_torque[1], jnt_cmd.security_torque[2], 
        jnt_cmd.security_torque[3], jnt_cmd.security_torque[4], jnt_cmd.security_torque[5]);
    printf("               (tor): %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", 
        jnt_cmd.control_torque[0], jnt_cmd.control_torque[1], jnt_cmd.control_torque[2], 
        jnt_cmd.control_torque[3], jnt_cmd.control_torque[4], jnt_cmd.control_torque[5]);
  }

  else {
    printf("Invalid command mode\n");
  }
  /////////////////////////////////////////////////////////////////////////////
  
  /////////////////////////////// Config commands ///////////////////////////////
  if(config_cmd.func_calls & URI_OPEN_REAL) {
    printf("Function call: robotinterface_open(0)\n");
    ur_state.robot_mode_id = 7;
  }

  else if(config_cmd.func_calls & URI_OPEN_SIMULATED)
    printf("Function call: robotinterface_open(1)\n");

  if(config_cmd.func_calls & URI_CLOSE) {
    printf("Function call: robotinterface_close()\n");
    ur_state.robot_mode_id = 8;
    ur_state.is_power_on_robot = 0;
  }

  if(config_cmd.func_calls & URI_UNLOCK_SECURITY_STOP)
    printf("Function call: robotinterface_unlock_security_stop()\n");

  if(config_cmd.func_calls & URI_SET_ROBOT_READY_MODE)
    printf("Function call: robotinterface_set_robot_ready_mode()\n");

  if(config_cmd.func_calls & URI_SET_ROBOT_RUNNING_MODE)
    printf("Function call: robotinterface_set_robot_running_mode()\n");

  if(config_cmd.func_calls & URI_POWER_ON_ROBOT) {
    printf("Function call: robotinterface_power_on_robot()\n");
    ur_state.robot_mode_id = 3;
    ur_state.is_power_on_robot = 1;
  }

  if(config_cmd.func_calls & URI_POWER_OFF_ROBOT) {
    printf("Function call: robotinterface_power_off_robot()\n");
    ur_state.robot_mode_id = 7;
    ur_state.is_power_on_robot = 0;
  }

  if(config_cmd.func_calls & URI_SET_TCP)
    printf("Function call: robotinterface_set_tcp([%.1f %.1f %.1f %.1f %.1f %.1f])\n",
           config_cmd.set_tcp_pose[0], config_cmd.set_tcp_pose[1], config_cmd.set_tcp_pose[2],
           config_cmd.set_tcp_pose[3], config_cmd.set_tcp_pose[4], config_cmd.set_tcp_pose[5]);

  if(config_cmd.func_calls & URI_SET_TCP_PAYLOAD_COG)
    printf("Function call: robotinterface_set_tcp_payload_cog([%.1f %.1f %.1f %.1f %.1f %.1f])\n",
           config_cmd.set_tcp_payload_cog[0], config_cmd.set_tcp_payload_cog[1],
           config_cmd.set_tcp_payload_cog[2], config_cmd.set_tcp_payload_cog[3],
           config_cmd.set_tcp_payload_cog[4], config_cmd.set_tcp_payload_cog[5]);

  if(config_cmd.func_calls & URI_SET_TCP_PAYLOAD)
    printf("Function call: robotinterface_set_tcp_payload(%.1f)\n", config_cmd.set_tcp_payload);

  if(config_cmd.func_calls & URI_SET_TCP_WRENCH)
    printf("Function call: robotinterface_set_tcp_wrench([%.1f %.1f %.1f %.1f %.1f %.1f], %d)\n",
           config_cmd.set_tcp_wrench[0], config_cmd.set_tcp_wrench[1], config_cmd.set_tcp_wrench[2], 
           config_cmd.set_tcp_wrench[3], config_cmd.set_tcp_wrench[4], config_cmd.set_tcp_wrench[5], 
           config_cmd.set_tcp_wrench_in_base_coord);
  
  if(config_cmd.func_calls & URI_SET_SECURITY_STOP)
    printf("Function call: robotinterface_security_stop(%d, %d, %d)\n", 
           config_cmd.security_stop_joint_code, config_cmd.security_stop_error_state, 
           config_cmd.security_stop_error_argument);
  /////////////////////////////////////////////////////////////////////////////
}

}
