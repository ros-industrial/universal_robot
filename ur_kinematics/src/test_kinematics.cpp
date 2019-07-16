#include <random>
#include <algorithm>
#include <cmath>

#include <ur_kinematics/ur_kin.hpp>

#include <gtest/gtest.h>

TEST(ConsistencyTests, pointer_array)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-M_PI*2.0, M_PI*2.0);

  double* q = new double[6]{0};
  double* T = new double[16]{0};
  std::array<double, 6> q_arr{0};
  std::array<double, 16> T_arr{0};

  for (auto i = 0; i < 500; ++i)
  {
    for(auto j = 0; j < 6; ++j)
    {
      auto q_val = dis(gen);
      q[j] = q_val;
      q_arr[j] = q_val;
    }

    ur_kinematics::forward(ur_kinematics::UR3, q, T);
    ur_kinematics::forward(ur_kinematics::UR3, q_arr, T_arr);

    for(auto j = 0; j < 6; ++j)
      EXPECT_EQ(T_arr[j], T[j]);
  }

  delete[] q;
  delete[] T;
}


TEST(ConsistencyTests, forward_inverse)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, M_PI*2.0);

  std::array<double, 6> q_arr{0};
  std::array<double, 16> T_arr{0};
  std::array<double, 48> q_sol{0};

  for (auto i = 0; i < 10000; ++i)
  {
    for(auto j = 0; j < 6; ++j)
    {
      auto q_val = dis(gen);
      q_arr[j] = q_val;
    }
    ur_kinematics::forward(ur_kinematics::UR3, q_arr, T_arr);
    auto solution_count = ur_kinematics::inverse(ur_kinematics::UR3, T_arr, q_sol, q_arr[5]);

    std::vector<std::array<double, 6>> solutions_as_vector;
    solutions_as_vector.reserve(solution_count);
    for(auto j = 0; j < solution_count; ++j)
    {
      std::array<double, 6> solution;
      auto start = std::begin(q_sol) + j*6;
      auto end = start + 6;
      std::copy(start, end, std::begin(solution));
      solutions_as_vector.emplace_back(solution);
    }

    constexpr std::array<std::pair<double, double>, 6> joint_limits{
      std::pair<double, double>(-2 * M_PI, 2 * M_PI),
      std::pair<double, double>(-2 * M_PI, 2 * M_PI),
      std::pair<double, double>(-2 * M_PI, 2 * M_PI),
      std::pair<double, double>(-2 * M_PI, 2 * M_PI),
      std::pair<double, double>(-2 * M_PI, 2 * M_PI),
      std::pair<double, double>(-2 * M_PI, 2 * M_PI)
    };

    auto expanded = ur_kinematics::expand_solutions(solutions_as_vector, joint_limits);
    auto found = std::find_if(std::begin(expanded), std::end(expanded), [q_arr](const std::array<double,6>& value)
    {
      for(int j = 0; j < 6; ++j)
      {
        if(std::abs(q_arr[j] - value[j]) > 1e-3)
        {
          return false;
        }
      }
      return true;
    }) != std::end(expanded);
    ASSERT_EQ(found, true);
  }
}




int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}