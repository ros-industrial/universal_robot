
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ur_ctrl_client/ur_robot_hw.h>

using namespace ur;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_controller_man");

  ros::AsyncSpinner spinner(1);
  spinner.start();


  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  std::string robot_ip;
  if(!nh_priv.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("Missing robot IP address (robot_ip)");
    return -1;
  }
  XmlRpc::XmlRpcValue v;
  if(!nh_priv.getParam("joint_names", v) || v.size() != 6) {
    ROS_ERROR("URRobotHW requires a list of the 6 joint names");
    return -1;
  }
  std::vector<std::string> joint_names;
  for(int i=0;i<6;i++)
    joint_names.push_back(v[i]);

  URRobotHW ur_hw(nh, joint_names);
  ur_hw.init(robot_ip);

  controller_manager::ControllerManager cm(&ur_hw, nh);

  ros::Duration period(1.0/125.0);
  while (ros::ok()) {
    ur_hw.read();
    cm.update(ros::Time::now(), period);
    ur_hw.write();
  }
}
