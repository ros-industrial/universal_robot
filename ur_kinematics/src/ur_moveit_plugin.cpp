/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Georgia Tech
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Georgia Tech nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Kelsey Hawkins */

/* Based on orignal source from Willow Garage. License copied below */

/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2012, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Sachin Chitta, David Lu!!, Ugo Cupcic */

#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

// UR kin
#include <ur_kinematics/ur_moveit_plugin.h>
#include <ur_kinematics/ur_kin.h>

// register URKinematicsPlugin as a KinematicsBase implementation
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(ur_kinematics::URKinematicsPlugin, kinematics::KinematicsBase)

namespace ur_kinematics
{

  URKinematicsPlugin::URKinematicsPlugin():active_(false) {}

void URKinematicsPlugin::getRandomConfiguration(KDL::JntArray &jnt_array, bool lock_redundancy) const
{
  std::vector<double> jnt_array_vector(dimension_, 0.0);
  state_->setToRandomPositions(joint_model_group_);
  state_->copyJointGroupPositions(joint_model_group_, &jnt_array_vector[0]);
  for (std::size_t i = 0; i < dimension_; ++i)
  {
    if (lock_redundancy)
      if (isRedundantJoint(i))
        continue;
    jnt_array(i) = jnt_array_vector[i];
  }
}

bool URKinematicsPlugin::isRedundantJoint(unsigned int index) const
{
  for (std::size_t j=0; j < redundant_joint_indices_.size(); ++j)
    if (redundant_joint_indices_[j] == index)
      return true;
  return false;
}

void URKinematicsPlugin::getRandomConfiguration(const KDL::JntArray &seed_state,
                                                 const std::vector<double> &consistency_limits,
                                                 KDL::JntArray &jnt_array,
                                                 bool lock_redundancy) const
{
  std::vector<double> values(dimension_, 0.0);
  std::vector<double> near(dimension_, 0.0);
  for (std::size_t i = 0 ; i < dimension_; ++i)
    near[i] = seed_state(i);

  // Need to resize the consistency limits to remove mimic joints
  std::vector<double> consistency_limits_mimic;
  for(std::size_t i = 0; i < dimension_; ++i)
  {
    if(!mimic_joints_[i].active)
      continue;
    consistency_limits_mimic.push_back(consistency_limits[i]);
  }

  joint_model_group_->getVariableRandomPositionsNearBy(state_->getRandomNumberGenerator(), values, near, consistency_limits_mimic);

  for (std::size_t i = 0; i < dimension_; ++i)
  {
    bool skip = false;
    if (lock_redundancy)
      for (std::size_t j = 0; j < redundant_joint_indices_.size(); ++j)
        if (redundant_joint_indices_[j] == i)
        {
          skip = true;
          break;
        }
    if (skip)
      continue;
    jnt_array(i) = values[i];
  }
}

bool URKinematicsPlugin::checkConsistency(const KDL::JntArray& seed_state,
                                           const std::vector<double> &consistency_limits,
                                           const KDL::JntArray& solution) const
{
  for (std::size_t i = 0; i < dimension_; ++i)
    if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
      return false;
  return true;
}

bool URKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model,
                                    const std::string& group_name,
                                    const std::string& base_frame,
                                    const std::vector<std::string>& tip_frames,
                                    double search_discretization)
{
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  const robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group)
    return false;

  if(!joint_model_group->isChain())
  {
    ROS_ERROR_NAMED("kdl","Group '%s' is not a chain", group_name.c_str());
    return false;
  }
  if(!joint_model_group->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("kdl","Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  KDL::Tree kdl_tree;

  if (!kdl_parser::treeFromUrdfModel(*robot_model.getURDF(), kdl_tree))
  {
    ROS_ERROR_NAMED("kdl","Could not initialize tree object");
    return false;
  }
  if (!kdl_tree.getChain(base_frame_, getTipFrame(), kdl_chain_))
  {
    ROS_ERROR_NAMED("kdl","Could not initialize chain object");
    return false;
  }

  dimension_ = joint_model_group->getActiveJointModels().size() + joint_model_group->getMimicJointModels().size();
  for (std::size_t i=0; i < joint_model_group->getJointModels().size(); ++i)
  {
    if(joint_model_group->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE || joint_model_group->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
    {
      ik_chain_info_.joint_names.push_back(joint_model_group->getJointModelNames()[i]);
      const std::vector<moveit_msgs::JointLimits> &jvec = joint_model_group->getJointModels()[i]->getVariableBoundsMsg();
      ik_chain_info_.limits.insert(ik_chain_info_.limits.end(), jvec.begin(), jvec.end());
    }
  }

  fk_chain_info_.joint_names = ik_chain_info_.joint_names;
  fk_chain_info_.limits = ik_chain_info_.limits;

  if(!joint_model_group->hasLinkModel(getTipFrame()))
  {
    ROS_ERROR_NAMED("kdl","Could not find tip name in joint group '%s'", group_name.c_str());
    return false;
  }
  ik_chain_info_.link_names.push_back(getTipFrame());
  fk_chain_info_.link_names = joint_model_group->getLinkModelNames();

  joint_min_.resize(ik_chain_info_.limits.size());
  joint_max_.resize(ik_chain_info_.limits.size());

  for(unsigned int i=0; i < ik_chain_info_.limits.size(); i++)
  {
    joint_min_(i) = ik_chain_info_.limits[i].min_position;
    joint_max_(i) = ik_chain_info_.limits[i].max_position;
  }

  // Get Solver Parameters
  int max_solver_iterations;
  double epsilon;
  bool position_ik;

  lookupParam("max_solver_iterations", max_solver_iterations, 500);
  lookupParam("epsilon", epsilon, 1e-5);
  lookupParam(group_name+"/position_only_ik", position_ik, false);

  if(position_ik)
    ROS_INFO_NAMED("kdl","Using position only ik");

  num_possible_redundant_joints_ = kdl_chain_.getNrOfJoints() - joint_model_group->getMimicJointModels().size() - (position_ik? 3:6);

  // Check for mimic joints
  bool has_mimic_joints = joint_model_group->getMimicJointModels().size() > 0;
  std::vector<unsigned int> redundant_joints_map_index;

  std::vector<kdl_kinematics_plugin::JointMimic> mimic_joints;
  unsigned int joint_counter = 0;
  for (std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    const robot_model::JointModel *jm = robot_model_->getJointModel(kdl_chain_.segments[i].getJoint().getName());

    //first check whether it belongs to the set of active joints in the group
    if (jm->getMimic() == NULL && jm->getVariableCount() > 0)
    {
      kdl_kinematics_plugin::JointMimic mimic_joint;
      mimic_joint.reset(joint_counter);
      mimic_joint.joint_name = kdl_chain_.segments[i].getJoint().getName();
      mimic_joint.active = true;
      mimic_joints.push_back(mimic_joint);
      ++joint_counter;
      continue;
    }
    if (joint_model_group->hasJointModel(jm->getName()))
    {
      if (jm->getMimic() && joint_model_group->hasJointModel(jm->getMimic()->getName()))
      {
        kdl_kinematics_plugin::JointMimic mimic_joint;
        mimic_joint.reset(joint_counter);
        mimic_joint.joint_name = kdl_chain_.segments[i].getJoint().getName();
        mimic_joint.offset = jm->getMimicOffset();
        mimic_joint.multiplier = jm->getMimicFactor();
        mimic_joints.push_back(mimic_joint);
        continue;
      }
    }
  }
  for (std::size_t i = 0; i < mimic_joints.size(); ++i)
  {
    if(!mimic_joints[i].active)
    {
      const robot_model::JointModel* joint_model = joint_model_group->getJointModel(mimic_joints[i].joint_name)->getMimic();
      for(std::size_t j=0; j < mimic_joints.size(); ++j)
      {
        if(mimic_joints[j].joint_name == joint_model->getName())
        {
          mimic_joints[i].map_index = mimic_joints[j].map_index;
        }
      }
    }
  }
  mimic_joints_ = mimic_joints;

  // Setup the joint state groups that we need
  state_.reset(new robot_state::RobotState(robot_model_));
  state_2_.reset(new robot_state::RobotState(robot_model_));

  // Store things for when the set of redundant joints may change
  position_ik_ = position_ik;
  joint_model_group_ = joint_model_group;
  max_solver_iterations_ = max_solver_iterations;
  epsilon_ = epsilon;

  lookupParam("arm_prefix", arm_prefix_, std::string(""));

  ur_joint_names_.push_back(arm_prefix_ + "shoulder_pan_joint");
  ur_joint_names_.push_back(arm_prefix_ + "shoulder_lift_joint");
  ur_joint_names_.push_back(arm_prefix_ + "elbow_joint");
  ur_joint_names_.push_back(arm_prefix_ + "wrist_1_joint");
  ur_joint_names_.push_back(arm_prefix_ + "wrist_2_joint");
  ur_joint_names_.push_back(arm_prefix_ + "wrist_3_joint");

  ur_link_names_.push_back(arm_prefix_ + "base_link");       // 0
  ur_link_names_.push_back(arm_prefix_ + "ur_base_link");    // 1
  ur_link_names_.push_back(arm_prefix_ + "shoulder_link");   // 2
  ur_link_names_.push_back(arm_prefix_ + "upper_arm_link");  // 3
  ur_link_names_.push_back(arm_prefix_ + "forearm_link");    // 4
  ur_link_names_.push_back(arm_prefix_ + "wrist_1_link");    // 5
  ur_link_names_.push_back(arm_prefix_ + "wrist_2_link");    // 6
  ur_link_names_.push_back(arm_prefix_ + "wrist_3_link");    // 7
  ur_link_names_.push_back(arm_prefix_ + "ee_link");         // 8

  ur_joint_inds_start_ = getJointIndex(ur_joint_names_[0]);

  // check to make sure the serial chain is properly defined in the model
  int cur_ur_joint_ind, last_ur_joint_ind = ur_joint_inds_start_;
  for(int i=1; i<6; i++) {
    cur_ur_joint_ind = getJointIndex(ur_joint_names_[i]);
    if(cur_ur_joint_ind < 0) {
      ROS_ERROR_NAMED("kdl",
        "Kin chain provided in model doesn't contain standard UR joint '%s'.",
        ur_joint_names_[i].c_str());
      return false;
    }
    if(cur_ur_joint_ind != last_ur_joint_ind + 1) {
      ROS_ERROR_NAMED("kdl",
        "Kin chain provided in model doesn't have proper serial joint order: '%s'.",
        ur_joint_names_[i].c_str());
      return false;
    }
    last_ur_joint_ind = cur_ur_joint_ind;
  }
  // if successful, the kinematic chain includes a serial chain of the UR joints

  kdl_tree.getChain(getBaseFrame(), ur_link_names_.front(), kdl_base_chain_);
  kdl_tree.getChain(ur_link_names_.back(), getTipFrame(), kdl_tip_chain_);

  // weights for redundant solution selection
  ik_weights_.resize(6);
  if(!lookupParam("ik_weights", ik_weights_, ik_weights_)) {
    ik_weights_[0] = 1.0;
    ik_weights_[1] = 1.0;
    ik_weights_[2] = 1.0;
    ik_weights_[3] = 1.0;
    ik_weights_[4] = 1.0;
    ik_weights_[5] = 1.0;
  }

  active_ = true;
  ROS_DEBUG_NAMED("kdl","KDL solver initialized");
  return true;
}

bool URKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int> &redundant_joints)
{
  if(num_possible_redundant_joints_ < 0)
  {
    ROS_ERROR_NAMED("kdl","This group cannot have redundant joints");
    return false;
  }
  if(redundant_joints.size() > num_possible_redundant_joints_)
  {
    ROS_ERROR_NAMED("kdl","This group can only have %d redundant joints", num_possible_redundant_joints_);
    return false;
  }
  std::vector<unsigned int> redundant_joints_map_index;
  unsigned int counter = 0;
  for(std::size_t i=0; i < dimension_; ++i)
  {
    bool is_redundant_joint = false;
    for(std::size_t j=0; j < redundant_joints.size(); ++j)
    {
      if(i == redundant_joints[j])
      {
        is_redundant_joint = true;
counter++;
        break;
      }
    }
    if(!is_redundant_joint)
    {
      // check for mimic
      if(mimic_joints_[i].active)
      {
redundant_joints_map_index.push_back(counter);
counter++;
      }
    }
  }
  for(std::size_t i=0; i < redundant_joints_map_index.size(); ++i)
    ROS_DEBUG_NAMED("kdl","Redundant joint map index: %d %d", (int) i, (int) redundant_joints_map_index[i]);

  redundant_joints_map_index_ = redundant_joints_map_index;
  redundant_joint_indices_ = redundant_joints;
  return true;
}

int URKinematicsPlugin::getJointIndex(const std::string &name) const
{
  for (unsigned int i=0; i < ik_chain_info_.joint_names.size(); i++) {
    if (ik_chain_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

int URKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
{
  int i=0;
  while (i < (int)kdl_chain_.getNrOfSegments()) {
    if (kdl_chain_.getSegment(i).getName() == name) {
      return i+1;
    }
    i++;
  }
  return -1;
}

bool URKinematicsPlugin::timedOut(const ros::WallTime &start_time, double duration) const
{
  return ((ros::WallTime::now()-start_time).toSec() >= duration);
}

bool URKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                        const std::vector<double> &ik_seed_state,
                                        std::vector<double> &solution,
                                        moveit_msgs::MoveItErrorCodes &error_code,
                                        const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          default_timeout_,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool URKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool URKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           const std::vector<double> &consistency_limits,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool URKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool URKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           const std::vector<double> &consistency_limits,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

typedef std::pair<int, double> idx_double;
bool comparator(const idx_double& l, const idx_double& r)
{ return l.second < r.second; }

bool URKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const std::vector<double> &consistency_limits,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_) {
    ROS_ERROR_NAMED("kdl","kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if(ik_seed_state.size() != dimension_) {
    ROS_ERROR_STREAM_NAMED("kdl","Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if(!consistency_limits.empty() && consistency_limits.size() != dimension_) {
    ROS_ERROR_STREAM_NAMED("kdl","Consistency limits be empty or must have size " << dimension_ << " instead of size " << consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  KDL::JntArray jnt_seed_state(dimension_);
  for(int i=0; i<dimension_; i++)
    jnt_seed_state(i) = ik_seed_state[i];

  solution.resize(dimension_);

  KDL::ChainFkSolverPos_recursive fk_solver_base(kdl_base_chain_);
  KDL::ChainFkSolverPos_recursive fk_solver_tip(kdl_tip_chain_);

  KDL::JntArray jnt_pos_test(jnt_seed_state);
  KDL::JntArray jnt_pos_base(ur_joint_inds_start_);
  KDL::JntArray jnt_pos_tip(dimension_ - 6 - ur_joint_inds_start_);
  KDL::Frame pose_base, pose_tip;

  KDL::Frame kdl_ik_pose;
  KDL::Frame kdl_ik_pose_ur_chain;
  double homo_ik_pose[4][4];
  double q_ik_sols[8][6]; // maximum of 8 IK solutions
  uint16_t num_sols;

  while(1) {
    if(timedOut(n1, timeout)) {
      ROS_DEBUG_NAMED("kdl","IK timed out");
      error_code.val = error_code.TIMED_OUT;
      return false;
    }

    /////////////////////////////////////////////////////////////////////////////
    // find transformation from robot base to UR base and UR tip to robot tip
    for(uint32_t i=0; i<jnt_pos_base.rows(); i++)
      jnt_pos_base(i) = jnt_pos_test(i);
    for(uint32_t i=0; i<jnt_pos_tip.rows(); i++)
      jnt_pos_tip(i) = jnt_pos_test(i + ur_joint_inds_start_ + 6);
    for(uint32_t i=0; i<jnt_seed_state.rows(); i++)
      solution[i] = jnt_pos_test(i);

    if(fk_solver_base.JntToCart(jnt_pos_base, pose_base) < 0) {
      ROS_ERROR_NAMED("kdl", "Could not compute FK for base chain");
      return false;
    }

    if(fk_solver_tip.JntToCart(jnt_pos_tip, pose_tip) < 0) {
      ROS_ERROR_NAMED("kdl", "Could not compute FK for tip chain");
      return false;
    }
    /////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////
    // Convert into query for analytic solver
    tf::poseMsgToKDL(ik_pose, kdl_ik_pose);
    kdl_ik_pose_ur_chain = pose_base.Inverse() * kdl_ik_pose * pose_tip.Inverse();

    kdl_ik_pose_ur_chain.Make4x4((double*) homo_ik_pose);
#if KDL_OLD_BUG_FIX
    // in older versions of KDL, setting this flag might be necessary
    for(int i=0; i<3; i++) homo_ik_pose[i][3] *= 1000; // strange KDL fix
#endif
    /////////////////////////////////////////////////////////////////////////////

    // Do the analytic IK
    num_sols = inverse((double*) homo_ik_pose, (double*) q_ik_sols,
                       jnt_pos_test(ur_joint_inds_start_+5));


    uint16_t num_valid_sols;
    std::vector< std::vector<double> > q_ik_valid_sols;
    for(uint16_t i=0; i<num_sols; i++)
    {
      bool valid = true;
      std::vector< double > valid_solution;
      valid_solution.assign(6,0.0);

      for(uint16_t j=0; j<6; j++)
      {
        if((q_ik_sols[i][j] <= ik_chain_info_.limits[j].max_position) && (q_ik_sols[i][j] >= ik_chain_info_.limits[j].min_position))
        {
          valid_solution[j] = q_ik_sols[i][j];
          valid = true;
          continue;
        }
        else if ((q_ik_sols[i][j] > ik_chain_info_.limits[j].max_position) && (q_ik_sols[i][j]-2*M_PI > ik_chain_info_.limits[j].min_position))
        {
          valid_solution[j] = q_ik_sols[i][j]-2*M_PI;
          valid = true;
          continue;
        }
        else if ((q_ik_sols[i][j] < ik_chain_info_.limits[j].min_position) && (q_ik_sols[i][j]+2*M_PI < ik_chain_info_.limits[j].max_position))
        {
          valid_solution[j] = q_ik_sols[i][j]+2*M_PI;
          valid = true;
          continue;
        }
        else
        {
          valid = false;
          break;
        }
      }

      if(valid)
      {
        q_ik_valid_sols.push_back(valid_solution);
      }
    }


    // use weighted absolute deviations to determine the solution closest the seed state
    std::vector<idx_double> weighted_diffs;
    for(uint16_t i=0; i<q_ik_valid_sols.size(); i++) {
      double cur_weighted_diff = 0;
      for(uint16_t j=0; j<6; j++) {
        // solution violates the consistency_limits, throw it out
        double abs_diff = std::fabs(ik_seed_state[ur_joint_inds_start_+j] - q_ik_valid_sols[i][j]);
        if(!consistency_limits.empty() && abs_diff > consistency_limits[ur_joint_inds_start_+j]) {
          cur_weighted_diff = std::numeric_limits<double>::infinity();
          break;
        }
        cur_weighted_diff += ik_weights_[j] * abs_diff;
      }
      weighted_diffs.push_back(idx_double(i, cur_weighted_diff));
    }

    std::sort(weighted_diffs.begin(), weighted_diffs.end(), comparator);

#if 0
    printf("start\n");
    printf("                     q %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f\n", ik_seed_state[1], ik_seed_state[2], ik_seed_state[3], ik_seed_state[4], ik_seed_state[5], ik_seed_state[6]);
    for(uint16_t i=0; i<weighted_diffs.size(); i++) {
      int cur_idx = weighted_diffs[i].first;
      printf("diff %f, i %d, q %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f\n", weighted_diffs[i].second, cur_idx, q_ik_valid_sols[cur_idx][0], q_ik_valid_sols[cur_idx][1], q_ik_valid_sols[cur_idx][2], q_ik_valid_sols[cur_idx][3], q_ik_valid_sols[cur_idx][4], q_ik_valid_sols[cur_idx][5]);
    }
    printf("end\n");
#endif

    for(uint16_t i=0; i<weighted_diffs.size(); i++) {
      if(weighted_diffs[i].second == std::numeric_limits<double>::infinity()) {
        // rest are infinity, no more feasible solutions
        break;
      }

      // copy the best solution to the output
      int cur_idx = weighted_diffs[i].first;
      solution = q_ik_valid_sols[cur_idx];

      // see if this solution passes the callback function test
      if(!solution_callback.empty())
        solution_callback(ik_pose, solution, error_code);
      else
        error_code.val = error_code.SUCCESS;

      if(error_code.val == error_code.SUCCESS) {
#if 0
        std::vector<std::string> fk_link_names;
        fk_link_names.push_back(ur_link_names_.back());
        std::vector<geometry_msgs::Pose> fk_poses;
        getPositionFK(fk_link_names, solution, fk_poses);
        KDL::Frame kdl_fk_pose;
        tf::poseMsgToKDL(fk_poses[0], kdl_fk_pose);
        printf("FK(solution) - pose \n");
        for(int i=0; i<4; i++) {
          for(int j=0; j<4; j++)
            printf("%1.6f ", kdl_fk_pose(i, j)-kdl_ik_pose(i, j));
          printf("\n");
        }
#endif
        return true;
      }
    }
    // none of the solutions were both consistent and passed the solution callback

    if(options.lock_redundant_joints) {
      ROS_DEBUG_NAMED("kdl","Will not pertubate redundant joints to find solution");
      break;
    }

    if(dimension_ == 6) {
      ROS_DEBUG_NAMED("kdl","No other joints to pertubate, cannot find solution");
      break;
    }

    // randomly pertubate other joints and try again
    if(!consistency_limits.empty()) {
      getRandomConfiguration(jnt_seed_state, consistency_limits, jnt_pos_test, false);
    } else {
      getRandomConfiguration(jnt_pos_test, false);
    }
  }

  ROS_DEBUG_NAMED("kdl","An IK that satisifes the constraints and is collision free could not be found");
  error_code.val = error_code.NO_IK_SOLUTION;
  return false;
}

bool URKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                        const std::vector<double> &joint_angles,
                                        std::vector<geometry_msgs::Pose> &poses) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_)
  {
    ROS_ERROR_NAMED("kdl","kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if(joint_angles.size() != dimension_)
  {
    ROS_ERROR_NAMED("kdl","Joint angles vector must have size: %d",dimension_);
    return false;
  }

  KDL::Frame p_out;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  KDL::JntArray jnt_pos_in(dimension_);
  for(unsigned int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);

  bool valid = true;
  for(unsigned int i=0; i < poses.size(); i++)
  {
    ROS_DEBUG_NAMED("kdl","End effector index: %d",getKDLSegmentIndex(link_names[i]));
    if(fk_solver.JntToCart(jnt_pos_in,p_out,getKDLSegmentIndex(link_names[i])) >=0)
    {
      tf::poseKDLToMsg(p_out,poses[i]);
    }
    else
    {
      ROS_ERROR_NAMED("kdl","Could not compute FK for %s",link_names[i].c_str());
      valid = false;
    }
  }
  return valid;
}

const std::vector<std::string>& URKinematicsPlugin::getJointNames() const
{
  return ik_chain_info_.joint_names;
}

const std::vector<std::string>& URKinematicsPlugin::getLinkNames() const
{
  return ik_chain_info_.link_names;
}

} // namespace
