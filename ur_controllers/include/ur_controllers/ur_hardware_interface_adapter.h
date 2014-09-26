
#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <ur_ctrl_client/pos_vel_acc_joint_iface.h>
#include <trajectory_interface/pos_vel_acc_state.h>

///**
// * \brief Adapter for a velocity-controlled hardware interface. Forwards desired velocities as commands.
// */
//template <class State>
//class HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, State>
//{
//public:
//  HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}
//
//  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
//  {
//    // Store pointer to joint handles
//    joint_handles_ptr_ = &joint_handles;
//
//    return true;
//  }
//
//  void starting(const ros::Time& time) {}
//  void stopping(const ros::Time& time) {}
//
//  void updateCommand(const ros::Time&     /*time*/,
//                     const ros::Duration& /*period*/,
//                     const State&         desired_state,
//                     const State&         /*state_error*/)
//  {
//    // Forward desired position to command
//    const unsigned int n_joints = joint_handles_ptr_->size();
//    for (unsigned int i = 0; i < n_joints; ++i) {(*joint_handles_ptr_)[i].setCommand(desired_state.velocity[i]);}
//  }
//
//private:
//  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
//};

// /**
//  * \brief Adapter for a position/velocity/acceleration-controlled hardware interface. 
//  * Forwards desired positions/velocities/accelerations as commands.
//  */
// template <class State>
// class HardwareInterfaceAdapter<hardware_interface::PosVelAccJointInterface, State>
// {
// public:
//   HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}
// 
//   bool init(std::vector<hardware_interface::PosVelAccJointHandle>& joint_handles, 
//                         ros::NodeHandle& controller_nh)
//   {
//     // Store pointer to joint handles
//     joint_handles_ptr_ = &joint_handles;
// 
//     return true;
//   }
// 
//   void starting(const ros::Time& time) {}
//   void stopping(const ros::Time& time) {}
// 
//   void updateCommand(const ros::Time&     /*time*/,
//                      const ros::Duration& /*period*/,
//                      const State&         desired_state,
//                      const State&         /*state_error*/)
//   {
//     // Forward desired position to command
//     const unsigned int n_joints = joint_handles_ptr_->size();
//     for (unsigned int i = 0; i < n_joints; ++i) {
//       (*joint_handles_ptr_)[i].setPosition(desired_state.position[i]);
//       (*joint_handles_ptr_)[i].setVelocity(desired_state.velocity[i]);
//       (*joint_handles_ptr_)[i].setAcceleration(desired_state.acceleration[i]);
//     }
//   }
// 
// private:
//   std::vector<hardware_interface::PosVelAccJointHandle>* joint_handles_ptr_;
// };
