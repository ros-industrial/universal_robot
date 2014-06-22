
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
// 
//   //~URKinematicsPlugin();

// 
//   virtual bool getPositionFK(const std::vector<std::string> &link_names,
//                              const std::vector<double> &joint_angles,
//                              std::vector<geometry_msgs::Pose> &poses) const;
// 
/**
* @brief Initialization function for the kinematics
* @return True if initialization was successful, false otherwise
*/
virtual bool initialize(const std::string& robot_description,
                        const std::string& group_name,
                        const std::string& base_frame,
                        const std::string& tip_frame,
                        double search_discretization);
// 
//   /**
// * @brief Return all the joint names in the order they are used internally
// */
//   const std::vector<std::string>& getJointNames() const;
// 
//   /**
// * @brief Return all the link names in the order they are represented internally
// */
//   const std::vector<std::string>& getLinkNames() const;
// 
protected:

  /**
* @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
* This particular method is intended for "searching" for a solutions by stepping through the redundancy
* (or other numerical routines).
* @param ik_pose the desired pose of the link
* @param ik_seed_state an initial guess solution for the inverse kinematics
* @param timeout The amount of time (in seconds) available to the solver
* @param solution the solution vector
* @param solution_callback A callback solution for the IK solution
* @param error_code an error code that encodes the reason for failure or success
* @param check_consistency Set to true if consistency check needs to be performed
* @param redundancy The index of the redundant joint
* @param consistency_limit The returned solutuion will contain a value for the redundant joint in the range [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
* @return True if a valid solution was found, false otherwise
*/
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          const IKCallbackFn &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const std::vector<double> &consistency_limits,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  std::vector<double> ik_weights_;
// 
//   moveit_msgs::KinematicSolverInfo fk_solver_info_, ik_solver_info_;
};
}
