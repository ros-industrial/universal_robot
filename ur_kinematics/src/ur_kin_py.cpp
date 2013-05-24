/*********************************************************************
 *
 * Python wrapper for UR kinematics
 * Author: Kelsey Hawkins (kphawkins@gatech.edu)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Georgia Institute of Technology
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

#include <boost/numpy.hpp>
#include <boost/scoped_array.hpp>

#include <ur_kinematics/ur_kin.h>

namespace p = boost::python;
namespace np = boost::numpy;

np::ndarray forward_wrapper(np::ndarray const & q_arr) {
  if(q_arr.get_dtype() != np::dtype::get_builtin<double>()) {
    PyErr_SetString(PyExc_TypeError, "Incorrect array data type");
    p::throw_error_already_set();
  }
  if(q_arr.get_nd() != 1) {
    PyErr_SetString(PyExc_TypeError, "Incorrect number of dimensions");
    p::throw_error_already_set();
  }
  if(q_arr.shape(0) != 6) {
    PyErr_SetString(PyExc_TypeError, "Incorrect shape (should be 6)");
    p::throw_error_already_set();
  }
  Py_intptr_t shape[2] = { 4, 4 };
  np::ndarray result = np::zeros(2,shape,np::dtype::get_builtin<double>()); 
  ur_kinematics::forward(reinterpret_cast<double*>(q_arr.get_data()), 
                         reinterpret_cast<double*>(result.get_data()));
  return result;
}

np::ndarray inverse_wrapper(np::ndarray const & array, PyObject * q6_des_py) {
  if(array.get_dtype() != np::dtype::get_builtin<double>()) {
    PyErr_SetString(PyExc_TypeError, "Incorrect array data type");
    p::throw_error_already_set();
  }
  if(array.get_nd() != 2) {
    PyErr_SetString(PyExc_TypeError, "Incorrect number of dimensions");
    p::throw_error_already_set();
  }
  if(array.shape(0) != 4 || array.shape(1) != 4) {
    PyErr_SetString(PyExc_TypeError, "Incorrect shape (should be 4x4)");
    p::throw_error_already_set();
  }
  double* T = reinterpret_cast<double*>(array.get_data());
  double* q_sols = (double*) malloc(8*6*sizeof(double));
  double q6_des = PyFloat_AsDouble(q6_des_py);
  int num_sols = ur_kinematics::inverse(T, q_sols, q6_des);
  q_sols = (double*) realloc(q_sols, num_sols*6*sizeof(double));
  return np::from_data(q_sols, np::dtype::get_builtin<double>() , p::make_tuple(num_sols, 6), p::make_tuple(6*sizeof(double), sizeof(double)), p::object());
}

BOOST_PYTHON_MODULE(ur_kin_py) {
  np::initialize();  // have to put this in any module that uses Boost.NumPy
  p::def("forward", forward_wrapper);
  p::def("inverse", inverse_wrapper);
}
