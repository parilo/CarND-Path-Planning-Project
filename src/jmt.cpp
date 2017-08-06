#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"

#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void generate_parameters(
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

int check_accelerate (
  double speed_limit,
  double T,
  double dt,
  const std::vector<double>& trajectory_params
) {

  double prev_v = trajectory_params[1];
  double t = dt;
  for (int i=0; t <= T; i++) {

    double t_degree;

    double coord_dot = trajectory_params[1];
    t_degree = t;
    for (int i=2; i<6; i++) {
      coord_dot += i * trajectory_params [i] * t_degree;
      t_degree *= t;
    }

    if(coord_dot >= speed_limit){
//std::cout << "speed limit exceeded t: " << t << std::endl;
      return -1;
    } else if (coord_dot < prev_v) {
//std::cout << "speed decreased t: " << t << std::endl;
      return 1;
    }

    prev_v = coord_dot;

    t += dt;
  }

  return 0;
}

int check_decelerate (
  double speed_lower_limit,
  double T,
  double dt,
  const std::vector<double>& trajectory_params
) {

  double prev_v = trajectory_params[1];
  double t = dt;
  for (int i=0; t <= T; i++) {

    double t_degree;

    double coord_dot = trajectory_params[1];
    t_degree = t;
    for (int i=2; i<6; i++) {
      coord_dot += i * trajectory_params [i] * t_degree;
      t_degree *= t;
    }

    if(coord_dot < speed_lower_limit){
//std::cout << "decelerate: speed limit exceeded t: " << t << std::endl;
      return 1;
    } else if (coord_dot > prev_v) {
//std::cout << "decelerate: speed decreased t: " << t << std::endl;
      return -1;
    }

    prev_v = coord_dot;

    t += dt;
  }

  return 0;
}

void optimize_accelerate_parameters (
  std::vector<double>& params,
  double T,
  double end_0,
  double start,
  double start_v,
  double start_a,
  double end_v,
  double end_a,
  double dt,
  std::function<void(std::vector<double>& params, std::vector<double> start, std::vector <double> end, double T)> generate_function,
  std::function<int(double, double, double, const std::vector<double>&)> check_function
)
{
  double S = end_0;
  int S_mod = 0;
  int prev_S_mod = S_mod;
  double S_step = 1.0;
  do {
    S += S_mod * S_step * 0.2;
    generate_function(params, {start, start_v, start_a}, {start + S, end_v, end_a}, T);
    S_mod = check_function(end_v, T, dt, params);

    if (prev_S_mod != S_mod && prev_S_mod != 0) {
      S_step *= 0.5;
    }
    prev_S_mod = S_mod;

//std::cout <<
//  "--- try accel: mod: " << S_mod <<
//  " S: " << S <<
//  " T: " << T <<
//  " S step: " << S_step <<
//  std::endl;
  } while (S_mod != 0 && S_step > 1e-3);
}

void generate_parameters_accelerate(
  std::vector<double>& params,
  double T,
  double start,
  double start_v,
  double start_a,
  double end_v,
  double end_a
)
{
  double S = 3 * end_v * T;
  double dt = T * 0.01;

  optimize_accelerate_parameters(
    params,
    T,
    S,
    start,
    start_v,
    start_a,
    end_v,
    end_a,
    dt,
    generate_parameters,
    check_accelerate
  );
}

void generate_parameters_decelerate(
  std::vector<double>& params,
  double T,
  double start,
  double start_v,
  double start_a,
  double end_v,
  double end_a
)
{
  double S = start_v * T;
  double dt = T * 0.01;

  optimize_accelerate_parameters(
    params,
    T,
    S,
    start,
    start_v,
    start_a,
    end_v,
    end_a,
    dt,
    generate_parameters,
    check_decelerate
  );
}



//int check_increase_coord (
////  double speed_limit,
//  double coord_limit,
//  double T,
//  double dt,
//  const std::vector<double>& trajectory_params
//) {

//  double prev_coord = trajectory_params[0];
//  double t = dt;
//  for (int i=0; t <= T; i++) {

//    double t_degree;

//    double coord = trajectory_params[0];
//    t_degree = t;
//    for (int i=1; i<6; i++) {
//      coord += trajectory_params [i] * t_degree;
//      t_degree *= t;
//    }

//    double coord_dot = trajectory_params[1];
//    t_degree = t;
//    for (int i=2; i<6; i++) {
//      coord_dot += i * trajectory_params [i] * t_degree;
//      t_degree *= t;
//    }

////    if(fabs(coord_dot) >= speed_limit){
////std::cout << "inc coord: speed limit exceeded t: " << t << std::endl;
////      return 1;
////    } else if (coord < prev_coord) {
////std::cout << "coord decreased t: " << t << std::endl;
////      return -1;
////    }

//    if(fabs(coord_dot) > 3){
//std::cout << "-- inc coord: speed limit exceeded t: " << t << std::endl;
//       return 1;
//    } else
//    if(coord >= coord_limit){
//std::cout << "-- inc coord: coord limit exceeded t: " << t << std::endl;
//      return 1;
//    } else if (coord < prev_coord) {
//std::cout << "-- inc coord: coord decreased t: " << t << std::endl;
//      return -1;
//    }

//    prev_coord = coord;

//    t += dt;
//  }

//  return 0;
//}

//int check_decrease_coord (
////  double speed_limit,
//  double coord_limit,
//  double T,
//  double dt,
//  const std::vector<double>& trajectory_params
//) {

//  double prev_coord = trajectory_params[0];
//  double t = dt;
//  for (int i=0; t <= T; i++) {

//    double t_degree;

//    double coord = trajectory_params[0];
//    t_degree = t;
//    for (int i=1; i<6; i++) {
//      coord += trajectory_params [i] * t_degree;
//      t_degree *= t;
//    }

//    double coord_dot = trajectory_params[1];
//    t_degree = t;
//    for (int i=2; i<6; i++) {
//      coord_dot += i * trajectory_params [i] * t_degree;
//      t_degree *= t;
//    }

////    if(fabs(coord_dot) >= speed_limit){
////std::cout << "inc coord: speed limit exceeded t: " << t << std::endl;
////      return 1;
////    } else if (coord > prev_coord) {
////std::cout << "coord decreased t: " << t << std::endl;
////      return -1;
////    }

//    if(fabs(coord_dot) > 3){
//std::cout << "-- dec coord: speed limit exceeded t: " << t << std::endl;
//       return 1;
//    } else
//    if(coord <= coord_limit){
//std::cout << "-- dec coord: coord limit exceeded t: " << t << std::endl;
//      return 1;
//    } else if (coord > prev_coord) {
//std::cout << "-- dec coord: coord increased t: " << t << std::endl;
//      return -1;
//    }

//    prev_coord = coord;

//    t += dt;
//  }

//  return 0;
//}

//void optimize_change_coord_parameters (
//  std::vector<double>& params,
//  double T_0,
//  double start,
//  double start_v,
//  double start_a,
//  double end,
//  double end_v,
//  double end_a,
//  std::function<void(std::vector<double>& params, std::vector<double> start, std::vector <double> end, double T)> generate_function,
//  std::function<int(double, double, double, const std::vector<double>&)> check_function
//)
//{
//  double T = T_0;
//  int T_mod = 0;
//  int prev_T_mod = T_mod;
//  double T_step = 1.0;
//  do {
//    T += T_mod * T_step * 0.2;
//    generate_function(params, {start, start_v, start_a}, {end, end_v, end_a}, T);
//    T_mod = check_function(end, T, T * 0.01, params);

//    if (prev_T_mod != T_mod && prev_T_mod != 0) {
//      T_step *= 0.5;
//    }
//    prev_T_mod = T_mod;

//std::cout << "--- try change coord: mod: " << T_mod << " T: " << T << std::endl;
//  } while (T_mod != 0 && T_step > 1e-4);
//}

//void generate_parameters_increase_coord(
//  std::vector<double>& params,
//  double& T,
//  double start,
//  double start_v,
//  double start_a,
//  double end,
//  double end_v,
//  double end_a
//)
//{
//  T = fabs(start - end) / 2.0;

//  optimize_change_coord_parameters(
//    params,
//    T,
//    start,
//    start_v,
//    start_a,
//    end,
//    end_v,
//    end_a,
//    generate_parameters,
//    check_increase_coord
//  );
//}

//void generate_parameters_decrease_coord(
//  std::vector<double>& params,
//  double& T,
//  double start,
//  double start_v,
//  double start_a,
//  double end,
//  double end_v,
//  double end_a
//)
//{
//  T = fabs(start - end) / 2.0;

//  optimize_change_coord_parameters(
//    params,
//    T,
//    start,
//    start_v,
//    start_a,
//    end,
//    end_v,
//    end_a,
//    generate_parameters,
//    check_decrease_coord
//  );
//}




int generate_points2(
  std::vector<double>& coords,
  std::vector<double>& coords_dot,
  std::vector<double>& coords_dot_dot,
  const std::vector<double>& trajectory_params,
  double T,
  double dt,
  int steps_count
)
{
  double t = 0;
  int i=0;
  for (; t < T && i < steps_count; i++) {

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

//    std::cout <<
//      "t: " << t <<
//      " x: " << coord <<
//      " v: " << coord_dot <<
//      " a: " << coord_dot_dot <<
//      std::endl;

    t += dt;
  }

  return i;
}

int JMT::generate_points_accelerate(
  std::vector<double>& coords,
  std::vector<double>& coords_dot,
  std::vector<double>& coords_dot_dot,
  double start,
  double start_v,
  double start_a,
  double end_v,
  double end_a,
  double dt,
  int steps_count
)
{
  double T = fabs(end_v - start_v) / 5.0;
  if (T < 2 * dt) return 0;

  std::vector<double> trajectory_params;
  if (start_v < end_v) {
    generate_parameters_accelerate(
      trajectory_params,
      T,
      start,
      start_v,
      start_a,
      end_v,
      end_a
    );
  } else {
    generate_parameters_decelerate(
      trajectory_params,
      T,
      start,
      start_v,
      start_a,
      end_v,
      end_a
    );
  }

  return generate_points2(
    coords,
    coords_dot,
    coords_dot_dot,
    trajectory_params,
    T,
    dt,
    steps_count
  );
}

//int JMT::generate_points_change_coord(
//  std::vector<double>& coords,
//  std::vector<double>& coords_dot,
//  std::vector<double>& coords_dot_dot,
//  double start,
//  double start_v,
//  double start_a,
//  double end,
//  double end_v,
//  double end_a,
//  double dt,
//  int steps_count
//)
//{
//  double T;
//  std::vector<double> trajectory_params;
//  if (start < end) {
//    generate_parameters_increase_coord(
//      trajectory_params,
//      T,
//      start,
//      start_v,
//      start_a,
//      end,
//      end_v,
//      end_a
//    );
//  } else {
//    generate_parameters_decrease_coord(
//      trajectory_params,
//      T,
//      start,
//      start_v,
//      start_a,
//      end,
//      end_v,
//      end_a
//    );
//  }

//  return generate_points2(
//    coords,
//    coords_dot,
//    coords_dot_dot,
//    trajectory_params,
//    T,
//    dt,
//    steps_count
//  );
//}















int JMT::generate_points(
  std::vector<double>& coords,
  std::vector<double>& coords_dot,
  std::vector<double>& coords_dot_dot,
  std::vector<double> start,
  std::vector<double> end,
  double T,
  int steps_count,
  double dt
){
  // coords.clear();
  // coords_dot.clear();
  // coords_dot_dot.clear();
  std::vector<double> trajectory_params;
  generate_parameters(trajectory_params, start, end, T);
  // std::cout << "params: " <<
  //   trajectory_params [0] << " " <<
  //   trajectory_params [1] << " " <<
  //   trajectory_params [2] << " " <<
  //   trajectory_params [3] << " " <<
  //   trajectory_params [4] << " " <<
  //   trajectory_params [5] << " " <<
  //   std::endl;

  return generate_points2(
    coords,
    coords_dot,
    coords_dot_dot,
    trajectory_params,
    T,
    dt,
    steps_count
  );

//  double t = 0;
//  for (int i=0; i<steps_count; i++) {

//    double t_degree;

//    double coord = trajectory_params[0];
//    t_degree = t;
//    for (int i=1; i<6; i++) {
//      coord += trajectory_params [i] * t_degree;
//      t_degree *= t;
//    }

//    double coord_dot = trajectory_params[1];
//    t_degree = t;
//    for (int i=2; i<6; i++) {
//      coord_dot += i * trajectory_params [i] * t_degree;
//      t_degree *= t;
//    }

//    double coord_dot_dot = 2 * trajectory_params[2];
//    t_degree = t;
//    for (int i=3; i<6; i++) {
//      coord_dot_dot += i * (i-1) * trajectory_params [i] * t_degree;
//      t_degree *= t;
//    }

//    coords.push_back(coord);
//    coords_dot.push_back(coord_dot);
//    coords_dot_dot.push_back(coord_dot_dot);

//    t += dt;

////    if (i < 5) {
////      std::cout <<
////        "t: " << t <<
////        " x: " << coord <<
////        " v: " << coord_dot <<
////        " a: " << coord_dot_dot <<
////        std::endl;
////    }
//  }
}
