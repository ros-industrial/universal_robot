
#include <ur_kinematics/ur_moveit_plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <ur_kinematics/ur_kin.h>
#include <tf_conversions/tf_kdl.h>
#include <limits>

//register URKinematicsPlugin as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(ur_kinematics::URKinematicsPlugin,kinematics::KinematicsBase);

namespace ur_kinematics
{
URKinematicsPlugin::URKinematicsPlugin() 
  : kdl_kinematics_plugin::KDLKinematicsPlugin(), ik_weights_(6) {}

bool URKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                          const std::vector<double> &ik_seed_state,
                                          double timeout,
                                          std::vector<double> &solution,
                                          const IKCallbackFn &solution_callback,
                                          moveit_msgs::MoveItErrorCodes &error_code,
                                          const std::vector<double> &consistency_limits,
                                          const kinematics::KinematicsQueryOptions &options) const
{
  KDL::Frame kdl_ik_pose;
  tf::poseMsgToKDL(ik_pose, kdl_ik_pose);
  double homo_ik_pose[4][4];
  kdl_ik_pose.Make4x4((double*) homo_ik_pose);
  for(int i=0; i<3; i++) homo_ik_pose[i][3] *= 1000;
  double q_ik_sols[8][6]; // maximum of 8 IK solutions
  uint16_t num_sols = inverse((double*) homo_ik_pose, (double*) q_ik_sols, ik_seed_state[5]);
  if(num_sols <= 0) {
    // NO SOLUTION
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  std::vector<int> tried_solutions;
  while(true) {
    // use weighted absolute deviations to determine the solution closest the seed state
    double min_weighted_diff = std::numeric_limits<double>::infinity();
    int min_diff_index = -1;
    for(uint16_t i=0; i<num_sols; i++) {
      if(std::find(tried_solutions.begin(), tried_solutions.end(), i) != tried_solutions.end())
        continue;

      double cur_weighted_diff = 0;
      for(uint16_t j=0; j<6; j++) {
        // solution violates the consistency_limits, throw it out
        if(std::fabs(ik_seed_state[j] - q_ik_sols[i][j]) > consistency_limits[j]) {
          cur_weighted_diff = std::numeric_limits<double>::infinity();
          break;
        }

        cur_weighted_diff += ik_weights_[j] * std::fabs(q_ik_sols[i][j] - ik_seed_state[j]);
      }
      if(cur_weighted_diff != std::numeric_limits<double>::infinity() &&
         cur_weighted_diff < min_weighted_diff) {
        min_weighted_diff = cur_weighted_diff;
        min_diff_index = i;
      }
    }

    if(min_diff_index < 0) {
      // NO SOLUTION, failed consistency_limits/solution_callback
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    // copy the best solution to the output
    solution.resize(6);
    std::copy(q_ik_sols[min_diff_index], q_ik_sols[min_diff_index+1], solution.begin());

    if(!solution_callback.empty())
      solution_callback(ik_pose, solution, error_code);
    else
      error_code.val = error_code.SUCCESS;

    if(error_code.val == error_code.SUCCESS) 
      return true;

    tried_solutions.push_back(min_diff_index);
  }
}

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

}
