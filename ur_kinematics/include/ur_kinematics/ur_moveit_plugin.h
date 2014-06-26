
#include <ros/ros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>

namespace ur_kinematics
{
class URKinematicsPlugin : public kdl_kinematics_plugin::KDLKinematicsPlugin
{
public:

/**
* @brief Plugin-able interface to the ur kinematics
*/
  URKinematicsPlugin();

  virtual bool initialize(const std::string& robot_description,
                          const std::string& group_name,
                          const std::string& base_frame,
                          const std::string& tip_frame,
                          double search_discretization);

protected:

  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const std::vector<double> &consistency_limits,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  std::vector<double> ik_weights_;

};
}
