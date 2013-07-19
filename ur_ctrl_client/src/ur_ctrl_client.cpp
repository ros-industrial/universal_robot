#include "ros/ros.h"
#include "industrial_robot_client/robot_state_interface.h"
#include "simple_message/simple_message.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/socket/udp_client.h"

#include "ur_ctrl_server/ur_ctrl_shared.h"
#include "ur_ctrl_client/jtp_msg_forwarder.h"

using industrial::tcp_client::TcpClient;
using industrial::udp_client::UdpClient;

int main(int argc, char** argv)
{
#ifndef TCP_COM
  UdpClient connection;
#else
  TcpClient connection;
#endif
  ros::init(argc, argv, "ur_ctrl_client");

  ros::NodeHandle nh_priv("~");
  std::string robot_ip;
  if(!nh_priv.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("Missing robot IP address (robot_ip)");
    return -1;
  }

  industrial::JTPMessageForwarder jtp_msg_fwd;

  ROS_INFO("Robot state connecting to IP address: %s", robot_ip.c_str());
  connection.init(const_cast<char *>(robot_ip.c_str()), UR_COM_PORT);
  jtp_msg_fwd.init(&connection);
  if(!connection.isConnected())
    connection.makeConnect();

  ros::spin();

  return 0;
}

