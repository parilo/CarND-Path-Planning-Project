#include <iostream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"

#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void JMT::generate_parameters(
  std::vector<double>& params,
  std::vector<double> start,
  std::vector <double> end,
  double T
){
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */

  Eigen::Matrix3f A;
  Eigen::Vector3f b;
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T, 5*T*T*T*T,
       6*T,   12*T*T,  20*T*T*T;
  b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T),
       end[1] - (start[1] + start[2]*T),
       end[2] - start[2];
  //cout << "Here is the matrix A:\n" << A << endl;
  //cout << "Here is the vector b:\n" << b << endl;
  Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);
  //cout << "The solution is:\n" << x << endl;

  params = {start[0], start[1], 0.5*start[2], x[0], x[1], x[2]};
}

void JMT::generate_points(
  std::vector<double>& coords,
  std::vector<double>& coords_dot,
  std::vector<double>& coords_dot_dot,
  std::vector<double> start,
  std::vector<double> end,
  double T,
  int steps_count,
  double dt
){
  coords.clear();
  coords_dot.clear();
  coords_dot_dot.clear();
  std::vector<double> trajectory_params;
  generate_parameters(trajectory_params, start, end, T);
  std::cout << "params: " <<
    trajectory_params [0] << " " <<
    trajectory_params [1] << " " <<
    trajectory_params [2] << " " <<
    trajectory_params [3] << " " <<
    trajectory_params [4] << " " <<
    trajectory_params [5] << " " <<
    std::endl;

  double t = 0;
  for (int i=0; i<steps_count; i++) {

    double t_degree;

    double coord = trajectory_params[0];
    t_degree = t;
    for (int i=1; i<6; i++) {
      coord += trajectory_params [i] * t_degree;
      t_degree *= t;
    }

    double coord_dot = trajectory_params[1];
    t_degree = t;
    for (int i=2; i<6; i++) {
      coord_dot += i * trajectory_params [i] * t_degree;
      t_degree *= t;
    }

    double coord_dot_dot = 2 * trajectory_params[2];
    t_degree = t;
    for (int i=3; i<6; i++) {
      coord_dot_dot += i * (i-1) * trajectory_params [i] * t_degree;
      t_degree *= t;
    }

    coords.push_back(coord);
    coords_dot.push_back(coord_dot);
    coords_dot_dot.push_back(coord_dot_dot);

    t += dt;

//    if (i < 5) {
//      std::cout <<
//        "t: " << t <<
//        " x: " << coord <<
//        " v: " << coord_dot <<
//        " a: " << coord_dot_dot <<
//        std::endl;
//    }
  }
}
