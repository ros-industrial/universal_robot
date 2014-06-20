
#include <ros/ros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/KinematicSolverInfo.h>

namespace ur_kinematics
{
class URKinematicsPlugin : public kinematics::KinematicsBase
{
public:

  /**
* @brief Plugin-able interface to the ur kinematics
*/
  URKinematicsPlugin();

  //~URKinematicsPlugin();

  virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                             const std::vector<double> &ik_seed_state,
                             std::vector<double> &solution,
                             moveit_msgs::MoveItErrorCodes &error_code,
                             const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                std::vector<double> &solution,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool getPositionFK(const std::vector<std::string> &link_names,
                             const std::vector<double> &joint_angles,
                             std::vector<geometry_msgs::Pose> &poses) const;

  /**
* @brief Initialization function for the kinematics
* @return True if initialization was successful, false otherwise
*/
  virtual bool initialize(const std::string& robot_description,
                          const std::string& group_name,
                          const std::string& base_frame,
                          const std::string& tip_frame,
                          double search_discretization);

  /**
* @brief Return all the joint names in the order they are used internally
*/
  const std::vector<std::string>& getJointNames() const;

  /**
* @brief Return all the link names in the order they are represented internally
*/
  const std::vector<std::string>& getLinkNames() const;

protected:

  moveit_msgs::KinematicSolverInfo fk_solver_info_, ik_solver_info_;
};
}
