#include <ur_kinematics/ur_moveit_plugin.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

void checkNear(const std::vector<double> &state_1,
               const std::vector<double> &state_2,
               const double epsilon)
{
  for (std::size_t i = 0; i < state_1.size(); ++i)
  {
    EXPECT_NEAR(state_1[i], state_2[i], epsilon);
  }
}

/** @brief Normalize the joint solution on [-pi, pi] */
std::vector<double> normalize(const std::vector<double> &joints)
{
  std::vector<double> out(joints.size());
  std::transform(joints.begin(), joints.end(), out.begin(), [](const double jv) {
    if (jv > M_PI)
      return -jv + M_PI;
    else if (jv < -M_PI)
      return -jv - M_PI;
    else
      return jv;
  });
  return out;
}

TEST(URKinematics, MoveItPluginTest)
{
  const std::string base_frame = "base_link";
  const std::string tool_frame = "ee_link";
  const std::string group = "manipulator";

  ur_kinematics::URKinematicsPlugin plugin;
  ASSERT_TRUE(plugin.initialize("robot_description", group, base_frame, tool_frame, 0.01));

  // Perform FK for an arbitrary joint state
  std::vector<double> joint_state = {0.0, -M_PI / 2.0, M_PI / 2.0, 0.0, M_PI / 2.0, 0.0};
  std::vector<geometry_msgs::Pose> poses;
  ASSERT_TRUE(plugin.getPositionFK({tool_frame}, joint_state, poses));
  ASSERT_EQ(poses.size(), 1);

  // Perform IK with the initial joint state as the seed to get a single joint state
  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  ASSERT_TRUE(plugin.getPositionIK(poses.front(), joint_state, solution, error_code));
  ASSERT_EQ(error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS);

  // Check that the normalized joint solution is close the initial state
  const double eps = 1.0e-6;
  checkNear(normalize(solution), joint_state, eps);

  // Perform IK with the initial joint state as the seed to get all valid joint states
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsResult result;
  ASSERT_TRUE(plugin.getPositionIK(poses, joint_state, solutions, result));
  ASSERT_EQ(solutions.size(), 8);

  // Make sure the first joint state is close to the initial state
  // The solutions should already be sorted by distance to the seed state, so compare only the first solution
  checkNear(normalize(solutions.front()), joint_state, eps);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "utest");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
