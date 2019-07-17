/*********************************************************************
 *
 * Benchmarking for forward and inverse kinematics for Univeral robot designs
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
#include <array>
#include <random>

#include <ur_kinematics/ur_kin.hpp>
#include <benchmark/benchmark.h>


static void BM_forward(benchmark::State& state) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-M_PI*2.0, M_PI*2.0);

  double* T = new double[16]{0};
  double* q = new double[6]{dis(gen)};

  for (auto _ : state)
  {
    ur_kinematics::forward(ur_kinematics::UR3, q, T);
    benchmark::ClobberMemory();
  }
  delete[] q;
  delete[] T;
}
BENCHMARK(BM_forward);

static void BM_forward_array(benchmark::State& state) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-M_PI*2.0, M_PI*2.0);

  std::array<double, 16> T{0};
  std::array<double, 6> q{dis(gen)};

  for (auto _ : state)
  {
    ur_kinematics::forward(ur_kinematics::UR3, q, T);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_forward_array);


static void BM_forward_all(benchmark::State& state) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-M_PI*2.0, M_PI*2.0);

  double* T1 = new double[16]{0};
  double* T2 = new double[16]{0};
  double* T3 = new double[16]{0};
  double* T4 = new double[16]{0};
  double* T5 = new double[16]{0};
  double* T6 = new double[16]{0};
  double* q = new double[6]{dis(gen)};

  for (auto _ : state)
  {
    ur_kinematics::forward_all(ur_kinematics::UR3, q, T1, T2, T3, T4, T5, T6);
    benchmark::ClobberMemory();
  }

  delete[] q;
  delete[] T1;
  delete[] T2;
  delete[] T3;
  delete[] T4;
  delete[] T5;
  delete[] T6;
}
BENCHMARK(BM_forward_all);

static void BM_forward_all_array(benchmark::State& state) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-M_PI*2.0, M_PI*2.0);

  std::array<double, 16> T1{dis(gen)};
  std::array<double, 16> T2{dis(gen)};
  std::array<double, 16> T3{dis(gen)};
  std::array<double, 16> T4{dis(gen)};
  std::array<double, 16> T5{dis(gen)};
  std::array<double, 16> T6{dis(gen)};
  std::array<double, 6> q{dis(gen)};

  for (auto _ : state)
  {
    ur_kinematics::forward_all(ur_kinematics::UR3, q, T1, T2, T3, T4, T5, T6);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_forward_all_array);


static void BM_inverse(benchmark::State& state) {
  double* T = new double[16]{0};
  double* q = new double[6]{0};
  double* q_sol = new double[48]{0};
  ur_kinematics::forward(ur_kinematics::UR3, q, T);

  for (auto _ : state)
    benchmark::DoNotOptimize(ur_kinematics::inverse(ur_kinematics::UR3, T, q_sol));

  delete[] q;
  delete[] q_sol;
  delete[] T;
}
BENCHMARK(BM_inverse);

static void BM_inverse_array(benchmark::State& state) {
  std::array<double, 16> T{0};
  std::array<double, 6> q{0};
  std::array<double, 48> q_sol{0};

  ur_kinematics::forward(ur_kinematics::UR3, q, T);

  for (auto _ : state)
    benchmark::DoNotOptimize(ur_kinematics::inverse(ur_kinematics::UR3, T, q_sol));
}
BENCHMARK(BM_inverse_array);

static void BM_enumerate_all(benchmark::State& state) {
  std::array<double, 16> T{0};
  std::array<double, 6> q{0};
  std::array<double, 48> q_sol{0};

  ur_kinematics::forward(ur_kinematics::UR3, q, T);
  auto solution_count = ur_kinematics::inverse(ur_kinematics::UR3, T, q_sol);

  std::vector<std::array<double, 6>> solutions_as_vector;
  solutions_as_vector.reserve(solution_count);
  for(auto i = 0; i < solution_count; ++i)
  {
    std::array<double, 6> solution;
    auto start = std::begin(q_sol) + i*6;
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

  for (auto _ : state)
    benchmark::DoNotOptimize(ur_kinematics::expand_solutions(solutions_as_vector, joint_limits));
}
BENCHMARK(BM_enumerate_all);

BENCHMARK_MAIN();
