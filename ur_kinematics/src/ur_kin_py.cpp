/*********************************************************************
 *
 * Python wrapper for UR kinematics
 * Authors: Kelsey Hawkins (kphawkins@gatech.edu)
 *          Leo Ghafari (leo@ascent.ai)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Georgia Institute of Technology
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
 *   * Neither the name of the Georgia Institute of Technology nor the names of
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

#include <ur_kinematics/ur_kin.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

std::array<double, 16> forward_wrapper(ur_kinematics::UR_PARAMS params, ur_kinematics::Solution<double> q) {
  std::array<double, 16> T;
  ur_kinematics::forward(params, q, T);
  return T;
}

ur_kinematics::ExpandedSolutionVec<double> inverse_wrapper(ur_kinematics::UR_PARAMS params, std::array<double, 16> T, ur_kinematics::JointsLimits<double> joint_limits,  double q6_des) {
  std::array<double, 48> solutions;
  auto solution_count = ur_kinematics::inverse(params, T, solutions, q6_des);

  ur_kinematics::SolutionVec<double> solutions_as_vector;
  solutions_as_vector.reserve(solution_count);
  for(auto i = 0; i < solution_count; ++i)
  {
    ur_kinematics::Solution<double> solution;
    auto start = std::begin(solutions) + i*6;
    auto end = start + 6;
    std::copy(start, end, std::begin(solution));
    solutions_as_vector.emplace_back(solution);
  }

  return ur_kinematics::expand_solutions(solutions_as_vector, joint_limits);
}

PYBIND11_MODULE(ur_kin_py, m)
{
  py::class_<ur_kinematics::UR_PARAMS>(m, "UR_PARAMS");

  m.attr("UR3") = ur_kinematics::UR3;
  m.attr("UR5") = ur_kinematics::UR5;
  m.attr("UR10") = ur_kinematics::UR10;
  m.attr("UR3E") = ur_kinematics::UR3E;
  m.attr("UR5E") = ur_kinematics::UR5E;
  m.attr("UR10E") = ur_kinematics::UR10E;


  m.def("forward", &forward_wrapper, "Forward kinematics");
  m.def("inverse", &inverse_wrapper, "Inverse kinematics");
}
