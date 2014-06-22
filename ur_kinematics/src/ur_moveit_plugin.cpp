
#include <ur_kinematics/ur_moveit_plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <ur_kinematics/ur_kin.h>
#include <tf_conversions/tf_kdl.h>
//#include <moveit/kdl_kinematics_plugin.cpp>

//register URKinematicsPlugin as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(ur_kinematics::URKinematicsPlugin,kinematics::KinematicsBase);

namespace ur_kinematics
{
  URKinematicsPlugin::URKinematicsPlugin() 
    : kdl_kinematics_plugin::KDLKinematicsPlugin(), ik_weights_(6) {}
//   //URKinematicsPlugin::~URKinematicsPlugin() {}

bool URKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const std::vector<double> &consistency_limits,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  //double homo_ik_pose[4][4];

  KDL::Frame kdl_ik_pose;
  tf::poseMsgToKDL(ik_pose, kdl_ik_pose);
  double homo_ik_pose[4][4];
  kdl_ik_pose.Make4x4((double*) homo_ik_pose);
  for(int i=0; i<3; i++) homo_ik_pose[i][3] *= 1000;
  double q_ik_sols[8][6];
  uint16_t num_sols = inverse((double*) homo_ik_pose, (double*) q_ik_sols, ik_seed_state[5]);
  if(num_sols <= 0) {
    // NO SOLUTION
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
  double min_weighted_diff = 1e9;
  int min_diff_index = -1;
  for(uint16_t i=0; i<num_sols; i++) {
    double cur_weighted_diff = 0;
    for(uint16_t j=0; j<6; j++) {
      cur_weighted_diff += ik_weights_[j] * std::fabs(q_ik_sols[i][j] - ik_seed_state[j]);
    }
    if(cur_weighted_diff < min_weighted_diff) {
      min_weighted_diff = cur_weighted_diff;
      min_diff_index = i;
    }
  }
  solution.resize(6);
  std::copy(q_ik_sols[min_diff_index], q_ik_sols[min_diff_index+1], solution.begin());

  if(!solution_callback.empty())
    solution_callback(ik_pose,solution,error_code);
  else
    error_code.val = error_code.SUCCESS;
  return true;
}
// 
//   bool URKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
//                                          const std::vector<double> &joint_angles,
//                                          std::vector<geometry_msgs::Pose> &poses) const
//   {
//     // double q[6];
//     // for(int i=0; i<6; i++)
//     //   q[i] = joint_angles[i];
//     // double T[6][16];
//     // forward_all(q, T[0], T[1], T[2], T[3], T[4], T[5]);
//     // for(int i=0; i<6; i++) {
//     //   aa
//     // }
//   }
// 
  /**
* @brief Initialization function for the kinematics
* @return True if initialization was successful, false otherwise
*/
bool URKinematicsPlugin::initialize(const std::string& robot_description,
                                    const std::string& group_name,
                                    const std::string& base_frame,
                                    const std::string& tip_frame,
                                    double search_discretization)
{ 
  ros::NodeHandle private_handle("~");
  if(private_handle.hasParam("ik_weights")) {
    private_handle.getParam("ik_weights", ik_weights_);
  } else {
    ik_weights_[0] = 1.0;
    ik_weights_[1] = 1.0;
    ik_weights_[2] = 0.1;
    ik_weights_[3] = 0.1;
    ik_weights_[4] = 0.3;
    ik_weights_[5] = 0.3;
  }

  return kdl_kinematics_plugin::KDLKinematicsPlugin::initialize(
           robot_description, group_name, base_frame, tip_frame, search_discretization);
}
// 
//   /**
// * @brief Return all the joint names in the order they are used internally
// */
//   const std::vector<std::string>& URKinematicsPlugin::getJointNames() const
//   {
//     return fk_solver_info_.joint_names;
//   }
// 
//   /**
// * @brief Return all the link names in the order they are represented internally
// */
//   const std::vector<std::string>& URKinematicsPlugin::getLinkNames() const
//   {
//     return fk_solver_info_.link_names;
//   }
}
