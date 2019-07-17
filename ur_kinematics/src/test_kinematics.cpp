/*********************************************************************
 *
 * Unit testing for forward and inverse kinematics for Univeral robot designs
 * Author: Leo Ghafari (leo@ascent.ai)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Ascent Robotics inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Ascent Robotics inc. nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
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
    auto found = false;

    for(const auto& expansion_set : expanded)
    {
      if(not found)
      {
        found = std::find_if(std::begin(expansion_set), std::end(expansion_set), [q_arr](const std::array<double,6>& value)
        {
          for(int j = 0; j < 6; ++j)
          {
            if(std::abs(q_arr[j] - value[j]) > 1e-3)
            {
              return false;
            }
          }
          return true;
        }) != std::end(expansion_set);
      }
    }

    ASSERT_EQ(found, true);
  }
}
