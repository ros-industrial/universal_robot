/*********************************************************************
 *
 * Provides forward and inverse kinematics for Univeral robot designs
 * Author: Kelsey Hawkins (kphawkins@gatech.edu)
 *         Leo Ghafari (leo@ascent.ai)
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
#ifndef UR_KIN_H
#define UR_KIN_H

// These kinematics find the tranfrom from the base link to the end effector.
// Though the raw D-H parameters specify a transform from the 0th link to the 6th link,
// offset transforms are specified in this formulation.
// To work with the raw D-H kinematics, use the inverses of the transforms below.

// Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1

#include <cmath>
#include <array>
#include <vector>
#include <map>
#include <type_traits>

namespace ur_kinematics {

  const double ZERO_THRESH = 1e-8;
  inline int SIGN(double x) {
    return (x > 0) - (x < 0);
  }

  // Denavit–Hartenberg parameters for calculations of kinematics of UR robots
  // Values for the existing UR robots are provided for simplicity and are obtained from:
  // https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/parameters-for-calculations-of-kinematics-and-dynamics-45257/
  struct UR_PARAMS
  {
    double d1;
    double a2;
    double a3;
    double d4;
    double d5;
    double d6;
  };

  constexpr UR_PARAMS UR3{0.1519, -0.24365, -0.21325, 0.11235, 0.08535, 0.0819};
  constexpr UR_PARAMS UR5{0.089159, -0.42500, -0.39225, 0.10915, 0.09465, 0.0823};
  constexpr UR_PARAMS UR10{ 0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922};

  constexpr UR_PARAMS UR3E{0.15185, -0.24355, -0.2132, 0.13105, 0.08535, 0.0921};
  constexpr UR_PARAMS UR5E{0.1625, -0.425, -0.3922, 0.1333, 0.0997, 0.0996};
  constexpr UR_PARAMS UR10E{ 0.1807, -0.6127, -0.57155, 0.17415, 0.11985, 0.11655};

  // @param p       The Denavit–Hartenberg parameters of the robot 
  // @param q       The 6 joint values (Must be pre-allocated)
  // @param T       The 4x4 end effector pose in row-major ordering (Must be pre-allocated)
  template<typename ValueType>
  inline void forward(const UR_PARAMS& p, const ValueType* const q, ValueType* const T) {
    const auto s1 = std::sin(q[0]);
    const auto c1 = std::cos(q[0]);

    const auto s2 = std::sin(q[1]);
    const auto c2 = std::cos(q[1]);

    const auto s3 = std::sin(q[2]);
    const auto c3 = std::cos(q[2]);

    const auto s5 = std::sin(q[4]);
    const auto c5 = std::cos(q[4]);

    const auto s6 = std::sin(q[5]);
    const auto c6 = std::cos(q[5]);


    const auto q234 = q[1] + q[2] + q[3];
    const auto s234 = std::sin(q234);
    const auto c234 = std::cos(q234);


    T[0] = ((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0;

    T[1] = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0);

    T[2] = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0));

    T[3] = ((p.d5*(s1*c234-c1*s234))/2.0 - (p.d5*(s1*c234+c1*s234))/2.0 -
          p.d4*s1 + (p.d6*(c1*c234-s1*s234)*s5)/2.0 + (p.d6*(c1*c234+s1*s234)*s5)/2.0 -
          p.a2*c1*c2 - p.d6*c5*s1 - p.a3*c1*c2*c3 + p.a3*c1*s2*s3);

    T[4] = c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0;

    T[5] = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) +
          s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0));

    T[6] = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) -
          s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0));

    T[7] = ((p.d5*(c1*c234-s1*s234))/2.0 - (p.d5*(c1*c234+s1*s234))/2.0 + p.d4*c1 +
          (p.d6*(s1*c234+c1*s234)*s5)/2.0 + (p.d6*(s1*c234-c1*s234)*s5)/2.0 + p.d6*c1*c5 -
          p.a2*c2*s1 - p.a3*c2*c3*s1 + p.a3*s1*s2*s3);

    T[8] = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0);

    T[9] = ((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6);

    T[10] = (s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0);

    T[11] = (p.d1 + (p.d6*(c234*c5-s234*s5))/2.0 + p.a3*(s2*c3+c2*s3) + p.a2*s2 -
         (p.d6*(c234*c5+s234*s5))/2.0 - p.d5*c234);

    T[12] = 0.0;
    T[13] = 0.0;
    T[14] = 0.0;
    T[15] = 1.0;
  }

  // @param p       The Denavit–Hartenberg parameters of the robot 
  // @param q       The 6 joint values 
  // @param T       The 4x4 end effector pose in row-major ordering
  template<typename ValueType>
  inline int forward(const UR_PARAMS& p, const std::array<ValueType, 6>& q , std::array<ValueType, 16>& T){
      forward(p, q.data(), T.data());
  }

  // @param p       The Denavit–Hartenberg parameters of the robot 
  // @param q       The 6 joint values (Must be pre-allocated)
  // @param Ti      The 4x4 link i pose in row-major ordering (Must be pre-allocated).
  template<typename ValueType>
  inline void forward_all(const UR_PARAMS& p, const ValueType* const q,
                          ValueType* const T1,
                          ValueType* const T2,
                          ValueType* const T3,
                          ValueType* const T4,
                          ValueType* const T5,
                          ValueType* const T6) {
    const auto s1 = std::sin(q[0]);
    const auto c1 = std::cos(q[0]);

    const auto s2 = std::sin(q[1]);
    const auto c2 = std::cos(q[1]);

    const auto s3 = std::sin(q[2]);
    const auto c3 = std::cos(q[2]);

    const auto s5 = std::sin(q[4]);
    const auto c5 = std::cos(q[4]);

    const auto s6 = std::sin(q[5]);
    const auto c6 = std::cos(q[5]);

    const auto q23 = q[1] + q[2];
    const auto q234 = q23 + q[3];

    const auto s23 = std::sin(q23);
    const auto c23 = std::cos(q23);
    const auto s234 = std::sin(q234);
    const auto c234 = std::cos(q234);

    T1[0] = c1;
    T1[1] = 0;
    T1[2] = s1;
    T1[3] = 0;
    T1[4] = s1;
    T1[5] = 0;
    T1[6] = -c1;
    T1[7] = 0;
    T1[8] = 0;
    T1[9] = 1;
    T1[10] = 0;
    T1[11] = p.d1;
    T1[12] = 0;
    T1[13] = 0;
    T1[14] = 0;
    T1[15] = 1;

    T2[0] = c1*c2;
    T2[1] = -c1*s2;
    T2[2] = s1;
    T2[3] = p.a2*c1*c2;
    T2[4] = c2*s1;
    T2[5] = -s1*s2;
    T2[6] = -c1;
    T2[7] = p.a2*c2*s1;
    T2[8] = s2;
    T2[9] = c2;
    T2[10] = 0;
    T2[11] = p.d1 + p.a2*s2;
    T2[12] = 0;
    T2[13] = 0;
    T2[14] = 0;
    T2[15] = 1;

    T3[0] = c23*c1;
    T3[1] = -s23*c1;
    T3[2] = s1;
    T3[3] = c1*(p.a3*c23 + p.a2*c2);
    T3[4] = c23*s1;
    T3[5] = -s23*s1;
    T3[6] = -c1;
    T3[7] = s1*(p.a3*c23 + p.a2*c2);
    T3[8] = s23;
    T3[9] = c23;
    T3[10] = 0;
    T3[11] = p.d1 + p.a3*s23 + p.a2*s2;
    T3[12] = 0;
    T3[13] = 0;
    T3[14] = 0;
    T3[15] = 1;

    T4[0] = c234*c1;
    T4[1] = s1;
    T4[2] = s234*c1;
    T4[3] = c1*(p.a3*c23 + p.a2*c2) + p.d4*s1;
    T4[4] = c234*s1;
    T4[5] = -c1;
    T4[6] = s234*s1;
    T4[7] = s1*(p.a3*c23 + p.a2*c2) - p.d4*c1;
    T4[8] = s234;
    T4[9] = 0;
    T4[10] = -c234;
    T4[11] = p.d1 + p.a3*s23 + p.a2*s2;
    T4[12] = 0;
    T4[13] = 0;
    T4[14] = 0;
    T4[15] = 1;

    T5[0] = s1*s5 + c234*c1*c5;
    T5[1] = -s234*c1;
    T5[2] = c5*s1 - c234*c1*s5;
    T5[3] = c1*(p.a3*c23 + p.a2*c2) + p.d4*s1 + p.d5*s234*c1;
    T5[4] = c234*c5*s1 - c1*s5;
    T5[5] = -s234*s1;
    T5[6] = -c1*c5 - c234*s1*s5;
    T5[7] = s1*(p.a3*c23 + p.a2*c2) - p.d4*c1 + p.d5*s234*s1;
    T5[8] = s234*c5;
    T5[9] = c234;
    T5[10] = -s234*s5;
    T5[11] = p.d1 + p.a3*s23 + p.a2*s2 - p.d5*c234;
    T5[12] = 0;
    T5[13] = 0;
    T5[14] = 0;
    T5[15] = 1;

    T6[0] = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6;
    T6[1] = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6;
    T6[2] = c5*s1 - c234*c1*s5;
    T6[3] = p.d6*(c5*s1 - c234*c1*s5) + c1*(p.a3*c23 + p.a2*c2) + p.d4*s1 + p.d5*s234*c1;
    T6[4] = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6;
    T6[5] = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1;
    T6[6] = -c1*c5 - c234*s1*s5;
    T6[7] = s1*(p.a3*c23 + p.a2*c2) - p.d4*c1 - p.d6*(c1*c5 + c234*s1*s5) + p.d5*s234*s1;
    T6[8] = c234*s6 + s234*c5*c6;
    T6[9] = c234*c6 - s234*c5*s6;
    T6[10] = -s234*s5;
    T6[11] = p.d1 + p.a3*s23 + p.a2*s2 - p.d5*c234 - p.d6*s234*s5;
    T6[12] = 0;
    T6[13] = 0;
    T6[14] = 0;
    T6[15] = 1;

  }

  // @param p       The Denavit–Hartenberg parameters of the robot 
  // @param q       The 6 joint values 
  // @param Ti      The 4x4 link i pose in row-major ordering.
  template<typename ValueType>
  inline void forward_all(const UR_PARAMS& p, const std::array<ValueType, 6>& q,
                          std::array<ValueType, 16>& T1,
                          std::array<ValueType, 16>& T2,
                          std::array<ValueType, 16>& T3,
                          std::array<ValueType, 16>& T4,
                          std::array<ValueType, 16>& T5,
                          std::array<ValueType, 16>& T6)
  {
    return forward_all(p, q.data(), T1.data(), T2.data(), T3.data(), T4.data(), T5.data(), T6.data());
  }

  // @param p       The Denavit–Hartenberg parameters of the robot 
  // @param T       The 4x4 end effector pose in row-major ordering (Must be pre-allocated)
  // @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI) (Must be pre-allocated)
  // @param q6_des  An optional parameter which designates what the q6 value should take
  //                in case of an infinite solution on that joint.
  // @return        Number of solutions found (maximum of 8)
  template<typename ValueType>
  inline int inverse(const UR_PARAMS& p, const ValueType* const T, ValueType* const q_sols, ValueType q6_des = 0.0) {
    int num_sols = 0;
    const auto T02 = -T[0];
    const auto T00 = T[1];
    const auto T01 = T[2];
    const auto T03 = -T[3];

    const auto T12 = -T[4];
    const auto T10 = T[5];
    const auto T11 = T[6];
    const auto T13 = -T[7];

    const auto T22 = T[8];
    const auto T20 = -T[9];
    const auto T21 = -T[10];
    const auto T23 = T[11];

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    ValueType q1[2];
    {
      const auto A = p.d6*T12 - T13;
      const auto B = p.d6*T02 - T03;
      const auto R = A*A + B*B;

      if(std::abs(A) < ZERO_THRESH) {
        ValueType div;

        if(std::abs(std::abs(p.d4) - std::abs(B)) < ZERO_THRESH)
          div = -SIGN(p.d4)*SIGN(B);
        else
          div = -p.d4/B;

        auto arcsin = std::asin(div);

        if(std::abs(arcsin) < ZERO_THRESH)
          arcsin = 0.0;

        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*M_PI;
        else
          q1[0] = arcsin;

        q1[1] = M_PI - arcsin;
      }
      else if(std::abs(B) < ZERO_THRESH) {
        ValueType div;

        if(std::abs(std::abs(p.d4) - std::abs(A)) < ZERO_THRESH)
          div = SIGN(p.d4)*SIGN(A);
        else
          div = p.d4/A;

        const auto arccos = std::acos(div);

        q1[0] = arccos;
        q1[1] = 2.0*M_PI - arccos;
      }
      else if(p.d4*p.d4 > R) {
        return num_sols;
      }
      else {
        const auto arccos = std::acos(p.d4 / sqrt(R)) ;
        const auto arctan = std::atan2(-B, A);
        auto pos = arccos + arctan;
        auto neg = -arccos + arctan;

        if(std::abs(pos) < ZERO_THRESH)
          pos = 0.0;

        if(std::abs(neg) < ZERO_THRESH)
          neg = 0.0;

        if(pos >= 0.0)
          q1[0] = pos;
        else
          q1[0] = 2.0*M_PI + pos;

        if(neg >= 0.0)
          q1[1] = neg;
        else
          q1[1] = 2.0*M_PI + neg;
      }
    }


    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    ValueType q5[2][2];
    {
      for(auto i=0; i<2; ++i) {
        const auto numer = (T03*std::sin(q1[i]) - T13*std::cos(q1[i])-p.d4);
        ValueType div;

        if(std::abs(std::abs(numer) - std::abs(p.d6)) < ZERO_THRESH)
          div = SIGN(numer) * SIGN(p.d6);
        else
          div = numer / p.d6;

        const auto arccos = std::acos(div);

        q5[i][0] = arccos;
        q5[i][1] = 2.0*M_PI - arccos;
      }
    }


    ////////////////////////////////////////////////////////////////////////////////

    {
      for(auto i=0; i<2; ++i) {
        for(auto j=0; j<2; ++j) {
          const auto c1 = std::cos(q1[i]);
          const auto s1 = std::sin(q1[i]);
          const auto c5 = std::cos(q5[i][j]);
          const auto s5 = std::sin(q5[i][j]);
          ValueType q6;

          ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if(std::abs(s5) < ZERO_THRESH)
            q6 = q6_des;
          else {
            q6 = std::atan2(SIGN(s5)*-(T01*s1 - T11*c1),
                       SIGN(s5)*(T00*s1 - T10*c1));

            if(std::abs(q6) < ZERO_THRESH)
              q6 = 0.0;

            if(q6 < 0.0)
              q6 += 2.0*M_PI;
          }
          ////////////////////////////////////////////////////////////////////////////////

          ValueType q2[2];
          ValueType q3[2];
          ValueType q4[2];

          ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          const auto c6 = std::cos(q6);
          const auto s6 = std::sin(q6);

          const auto x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
          const auto x04y = c5*(T20*c6 - T21*s6) - T22*s5;
          const auto p13x = p.d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - p.d6*(T02*c1 + T12*s1) +
                        T03*c1 + T13*s1;
          const auto p13y = T23 - p.d1 - p.d6*T22 + p.d5*(T21*c6 + T20*s6);
          auto c3 = (p13x*p13x + p13y*p13y - p.a2*p.a2 - p.a3*p.a3) / (2.0*p.a2*p.a3);

          if(std::abs(std::abs(c3) - 1.0) < ZERO_THRESH)
            c3 = SIGN(c3);
          else if(std::abs(c3) > 1.0) {
            // TODO NO SOLUTION
            continue;
          }

          const auto arccos = std::acos(c3);
          q3[0] = arccos;
          q3[1] = 2.0*M_PI - arccos;

          const auto denom = p.a2*p.a2 + p.a3*p.a3 + 2*p.a2*p.a3*c3;
          const auto s3 = std::sin(arccos);
          const auto A = (p.a2 + p.a3*c3);
          const auto B = p.a3*s3;

          q2[0] = std::atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
          q2[1] = std::atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);

          const auto c23_0 = std::cos(q2[0]+q3[0]);
          const auto s23_0 = std::sin(q2[0]+q3[0]);
          const auto c23_1 = std::cos(q2[1]+q3[1]);
          const auto s23_1 = std::sin(q2[1]+q3[1]);

          q4[0] = std::atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
          q4[1] = std::atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);

          ////////////////////////////////////////////////////////////////////////////////
          for(auto k=0; k<2; ++k) {
            if(std::abs(q2[k]) < ZERO_THRESH)
              q2[k] = 0.0;
            else if(q2[k] < 0.0)
              q2[k] += 2.0*M_PI;

            if(std::abs(q4[k]) < ZERO_THRESH)
              q4[k] = 0.0;
            else if(q4[k] < 0.0)
              q4[k] += 2.0*M_PI;

            q_sols[num_sols*6+0] = q1[i];
            q_sols[num_sols*6+1] = q2[k];
            q_sols[num_sols*6+2] = q3[k];
            q_sols[num_sols*6+3] = q4[k];
            q_sols[num_sols*6+4] = q5[i][j];
            q_sols[num_sols*6+5] = q6;

            num_sols++;
          }

        }
      }
    }


    return num_sols;
  }

  // @param p       The Denavit–Hartenberg parameters of the robot 
  // @param T       The 4x4 end effector pose in row-major ordering
  // @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
  // @param q6_des  An optional parameter which designates what the q6 value should take
  //                in case of an infinite solution on that joint.
  // @return        Number of solutions found (maximum of 8)
  template<typename ValueType>
  inline int inverse(const UR_PARAMS& p, const std::array<ValueType, 16>& T , std::array<ValueType, 48>& q_sols, ValueType q6_des = 0.0){
      return inverse(p, T.data(), q_sols.data(), q6_des);
  }


  constexpr auto joints_count = 6u;

  template<typename ValueType>
  using Solution = std::array<ValueType, joints_count>;

  template<typename ValueType>
  using SolutionVec = std::vector<Solution<ValueType>>;

  template<typename ValueType>
  using ExpandedSolutionVec = std::vector<SolutionVec<ValueType>>;

  template<typename ValueType>
  using Limit = std::pair<ValueType, ValueType>;

  template<typename ValueType>
  using JointsLimits = std::array<Limit<ValueType>, joints_count>;

  // @param initial_solution_set    Vector of initial solutions for an end effector position
  // @param joints_limit            Joints limits of the robot
  // @param partial_solution        Current partial solution
  // @param expanded_solutions      List of expanded solutions
  // @param current_joint           Joint id being currently rotated
  template <typename ValueType>
  inline void expand_solution(const Solution<ValueType>& initial_solution, const JointsLimits<ValueType>& joints_limit, Solution<ValueType>& partial_solution, SolutionVec<ValueType>& expanded_solutions, std::size_t current_joint)
  {
    if(current_joint == joints_count)
    {
      expanded_solutions.emplace_back(partial_solution);
    }
    else
    {
      auto q = initial_solution[current_joint];
      if(joints_limit[current_joint].first <= q && q <= joints_limit[current_joint].second)
      {
        partial_solution[current_joint] = q;
        expand_solution(initial_solution, joints_limit, partial_solution, expanded_solutions, current_joint+1);
      }

      q = initial_solution[current_joint] + 2.0*M_PI;
      while(q <= joints_limit[current_joint].second)
      {
        partial_solution[current_joint] = q;
        q += 2.0*M_PI;
        expand_solution(initial_solution, joints_limit, partial_solution, expanded_solutions, current_joint+1);
      }

      q = initial_solution[current_joint] - 2.0*M_PI;
      while(q >= joints_limit[current_joint].first)
      {
        partial_solution[current_joint] = q;
        q -= 2.0*M_PI;
        expand_solution(initial_solution, joints_limit, partial_solution, expanded_solutions, current_joint+1);
      }
    }
  }

  // @param initial_solution_set    Vector of initial solutions for an end effector position
  // @param joints_limit            Joints limits of the robot
  // @return                        The list of all solutions for the end effector position
  template <typename ValueType>
  inline ExpandedSolutionVec<double> expand_solutions(const SolutionVec<ValueType>& initial_solution_set, const JointsLimits<ValueType>& joints_limit)
  {
    ExpandedSolutionVec<double> expanded_solutions;
    Solution<ValueType> partial{0};
    for(const auto& sol : initial_solution_set)
    {
      SolutionVec<ValueType> current_expansion;
      expand_solution(sol, joints_limit, partial, current_expansion, 0);
      expanded_solutions.emplace_back(current_expansion);
    }
    return expanded_solutions;
  }

}
#endif //UR_KIN_H
