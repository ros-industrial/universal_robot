
#define LINUXSOCKETS
#define FLOAT64

#include "ur_ctrl_server/ur_ctrl_shared.h"
#ifdef TCP_COM
#include "simple_message/socket/tcp_server.h"
typedef industrial::tcp_server::TcpServer SimpleServer;
#else
#include "simple_message/socket/udp_server.h"
typedef industrial::udp_server::UdpServer SimpleServer;
#endif

template <class T>
int runController(int argc, char** argv)
{

  SimpleServer connection;
  connection.init(UR_COM_PORT);
  T ur_ctrl(&connection);

  if(ur_ctrl.initRobot(argc, argv))
    return -1;
  ur_ctrl.controlLoop();

  return 0;
}
