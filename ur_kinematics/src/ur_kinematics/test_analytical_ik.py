#########################################################################
#
# Basic test of the Python wrapper for UR kinematics
# Author: Leo Ghafari (leo@ascent.ai)
#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2019, Ascent Robotics inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Ascent Robotics inc. nor the names of
#     its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#########################################################################
import numpy as np
import sys
from ur_kinematics.ur_kin_py import forward, inverse, UR3
import roslib
roslib.load_manifest("ur_kinematics")


def best_sol(sols, q_guess, weights):
    valid_sols = []
    for sol in sols:
        test_sol = np.ones(6)*9999.
        for i in range(6):
            for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
                test_ang = sol[i] + add_ang
                if (abs(test_ang) <= 2.*np.pi and 
                    abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                    test_sol[i] = test_ang
        if np.all(test_sol != 9999.):
            valid_sols.append(test_sol)
    if len(valid_sols) == 0:
        return None
    best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2, 1))
    return valid_sols[best_sol_ind]


def test_q(q):
    x = forward(UR3, q)
    sols = inverse(UR3, np.array(x), float(q[5])).reshape(8, 6)

    qsol = best_sol(sols, q, [1.]*6)
    if qsol is None:
        qsol = [999.]*6
    diff = np.sum(np.abs(np.array(qsol) - q))
    if diff > 0.001:
        print np.array(sols)
        print 'Best q:', qsol
        print 'Actual:', np.array(q)
        print 'Diff:  ', q - qsol
        print 'Difdiv:', (q - qsol)/np.pi
        if raw_input() == 'q':
            sys.exit()


def main():
    np.set_printoptions(precision=3)
    print "Testing multiples of pi/2..."
    for i1 in range(0,5):
        for i2 in range(0,5):
            print i1, i2
            for i3 in range(0,5):
                for i4 in range(0,5):
                    for i5 in range(0,5):
                        for i6 in range(0,5):
                            q = np.array([i1*np.pi/2., i2*np.pi/2., i3*np.pi/2., 
                                          i4*np.pi/2., i5*np.pi/2., i6*np.pi/2.])
                            test_q(q)
    print "Testing random configurations..."
    for i in range(10000):
        q = (np.random.rand(6)-.5)*4*np.pi
        test_q(q)
    print "Done!"


if __name__ == "__main__":
    main()
