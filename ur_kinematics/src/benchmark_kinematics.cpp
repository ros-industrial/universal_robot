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
