import numpy as np
import sys
import roslib
roslib.load_manifest("ur_kinematics")
from ur_kin_py import forward, inverse

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
    best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2,1))
    return valid_sols[best_sol_ind]

def test_q(q):
    x = forward(q)
    sols = inverse(np.array(x), float(q[5]))

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
        print i1-3, i2-3, i3-3, i4-3, i5-3, i6-3
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
    if False:
        import cProfile
        cProfile.run('main()', 'ik_prof')
    else:
        main()
