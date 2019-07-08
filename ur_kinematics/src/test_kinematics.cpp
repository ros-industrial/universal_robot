#include <array>

#include <ur_kinematics/ur_kin.h>
#include <ur_kinematics/ur_kin_v2.hpp>


#include <benchmark/benchmark.h>

static void BM_forward(benchmark::State& state) {
    double* T = new double[16]{0};
    double* q = new double[6]{0};

    for (auto _ : state)
        ur_kinematics::forward(q, T);

    delete[] q;
    delete[] T;
}
BENCHMARK(BM_forward);

static void BM_forward_v2(benchmark::State& state) {
    double* T = new double[16]{0};
    double* q = new double[6]{0};

    for (auto _ : state)
        ur_kinematics_v2::forward(ur_kinematics_v2::UR3, q, T);

    delete[] q;
    delete[] T;
}
BENCHMARK(BM_forward_v2);

static void BM_forward_v2_array(benchmark::State& state) {
    std::array<double, 16> T{0};
    std::array<double, 6> q{0};

    for (auto _ : state)
        ur_kinematics_v2::forward(ur_kinematics_v2::UR3, q, T);
}
BENCHMARK(BM_forward_v2_array);


static void BM_forward_all(benchmark::State& state) {
    double* T1 = new double[16]{0};
    double* T2 = new double[16]{0};
    double* T3 = new double[16]{0};
    double* T4 = new double[16]{0};
    double* T5 = new double[16]{0};
    double* T6 = new double[16]{0};
    double* q = new double[6]{0};

    for (auto _ : state)
        ur_kinematics::forward_all(q, T1, T2, T3, T4, T5, T6);

    delete[] q;
    delete[] T1;
    delete[] T2;
    delete[] T3;
    delete[] T4;
    delete[] T5;
    delete[] T6;
}
BENCHMARK(BM_forward_all);

static void BM_forward_all_v2(benchmark::State& state) {
    double* T1 = new double[16]{0};
    double* T2 = new double[16]{0};
    double* T3 = new double[16]{0};
    double* T4 = new double[16]{0};
    double* T5 = new double[16]{0};
    double* T6 = new double[16]{0};
    double* q = new double[6]{0};

    for (auto _ : state)
        ur_kinematics_v2::forward_all(ur_kinematics_v2::UR3, q, T1, T2, T3, T4, T5, T6);

    delete[] q;
    delete[] T1;
    delete[] T2;
    delete[] T3;
    delete[] T4;
    delete[] T5;
    delete[] T6;
}
BENCHMARK(BM_forward_all_v2);

static void BM_forward_all_v2_array(benchmark::State& state) {
    std::array<double, 16> T1{0};
    std::array<double, 16> T2{0};
    std::array<double, 16> T3{0};
    std::array<double, 16> T4{0};
    std::array<double, 16> T5{0};
    std::array<double, 16> T6{0};
    std::array<double, 6> q{0};

    for (auto _ : state)
        ur_kinematics_v2::forward_all(ur_kinematics_v2::UR3, q, T1, T2, T3, T4, T5, T6);
}
BENCHMARK(BM_forward_all_v2_array);


static void BM_inverse(benchmark::State& state) {
    double* T = new double[16]{0};
    double* q = new double[6]{0};
    double* q_sol = new double[48]{0};
    
    ur_kinematics::forward(q, T);
    for (auto _ : state)
        ur_kinematics::inverse(T, q_sol);

    delete[] q;
    delete[] q_sol;
    delete[] T;
}
BENCHMARK(BM_inverse);

static void BM_inverse_v2(benchmark::State& state) {
    double* T = new double[16]{0};
    double* q = new double[6]{0};
    double* q_sol = new double[48]{0};
    ur_kinematics_v2::forward(ur_kinematics_v2::UR3, q, T);

    for (auto _ : state)
        ur_kinematics_v2::inverse(ur_kinematics_v2::UR3, T, q_sol);

    delete[] q;
    delete[] q_sol;
    delete[] T;
}
BENCHMARK(BM_inverse_v2);

static void BM_inverse_v2_array(benchmark::State& state) {
    std::array<double, 16> T{0};
    std::array<double, 6> q{0};
    std::array<double, 48> q_sol{0};

    ur_kinematics_v2::forward(ur_kinematics_v2::UR3, q, T);

    for (auto _ : state)
        ur_kinematics_v2::inverse(ur_kinematics_v2::UR3, T, q_sol);
}
BENCHMARK(BM_inverse_v2_array);



BENCHMARK_MAIN();
