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

/**
 * @brief Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

 * @param params - output array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

 * @param start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]
 * @param end - the desired end state for vehicle. Like "start" this is a
      length three array. [e, e_dot, e_double_dot]
 * @param T - The duration, in seconds, over which this maneuver should occur.
 */
void generate_parameters(
  std::vector<double>& params,
  std::vector<double> start,
  std::vector<double> end,
  double T
){

  // Eigen example of solving linear system
  Eigen::Matrix3f A;
  Eigen::Vector3f b;
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T, 5*T*T*T*T,
       6*T,   12*T*T,  20*T*T*T;
  b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T),
       end[1] - (start[1] + start[2]*T),
       end[2] - start[2];
  Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);

  params = {start[0], start[1], 0.5*start[2], x[0], x[1], x[2]};
}

/**
 * @brief Check acceleration trajectory wether it violates constraints
 * @param speed_limit - trajectory must not exceed speed_limit
 * @param T - trajectory duration
 * @param dt - time step
 * @param trajectory_params - JMT params
 * @return 0 - if trajectory is good enough, -1 or 1 - direction of S modification for optimizer
 */
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
      return -1;
    } else if (coord_dot < prev_v) {
      return 1;
    }

    prev_v = coord_dot;

    t += dt;
  }

  return 0;
}

/**
 * @brief Check acceleration trajectory wether it violates constraints
 * @param speed_lower_limit - trajectory must not go under speed_lower_limit
 * @param T - trajectory duration
 * @param dt - time step
 * @param trajectory_params - JMT params
 * @return 0 - if trajectory is good enough, -1 or 1 - direction of S modification for optimizer
 */
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
      return 1;
    } else if (coord_dot > prev_v) {
      return -1;
    }

    prev_v = coord_dot;

    t += dt;
  }

  return 0;
}

/**
 * @brief Optimize JMT parameters for acceleration and deceleration
 * @param params - output JMT parameters
 * @param T - trajectory time
 * @param end_0 - begin point for trajectory size optimization
 * @param start - start coordinate
 * @param start_v - start first derivative
 * @param start_a - start second derivative
 * @param end_v - end derivative
 * @param end_a - end second derivative
 * @param dt - time parameter step for points gereration
 * @param generate_function - calc JMT params function. see generate_parameters func
 * @param check_function - check trajectory function
 */
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

  } while (S_mod != 0 && S_step > 1e-3);
}

/**
 * @brief Calculates optimal JMT parameters for acceleration
 * @param params - output JMT parameters
 * @param T - trajectory time
 * @param start - start coordinate
 * @param start_v - start first derivative
 * @param start_a - start second derivative
 * @param end_v - end derivative
 * @param end_a - end second derivative
 */
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

/**
 * @brief Calculates optimal JMT parameters for deceleration
 * @param params - output JMT parameters
 * @param T - trajectory time
 * @param start - start coordinate
 * @param start_v - start first derivative
 * @param start_a - start second derivative
 * @param end_v - end derivative
 * @param end_a - end second derivative
 */
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

/**
 * @brief Calculate trajectory points for JMT parameters
   * @param coords - output coordinates
   * @param coords_dot - output first derivative of coordinates
   * @param coords_dot_dot - output second derivative
   * @param trajectory_params - JMT parameters
   * @param T - trajectory time
   * @param dt - time parameter step for points gereration
   * @param steps_count - maximum amount of steps to generate
 * @return
 */
int generate_points(
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

    t += dt;
  }

  return i;
}

/**
 * @brief Generate points for JMT for acceleration and deceleration
 * @param coords - output coordinates
 * @param coords_dot - output first derivative of coordinates
 * @param coords_dot_dot - output second derivative
 * @param start - start coordinate
 * @param start_v - start first derivative
 * @param start_a - start second derivative
 * @param end_v - end derivative
 * @param end_a - end second derivative
 * @param dt - time parameter step for points gereration
 * @param steps_count - maximum amount of steps to generate
 * @return generated steps, may vary because trajectory may have different time length
 */
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
  double T = fabs(end_v - start_v) / 4.0;
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

  return generate_points(
    coords,
    coords_dot,
    coords_dot_dot,
    trajectory_params,
    T,
    dt,
    steps_count
  );
}

/**
 * @brief Generate points for JMT for changing coorinate trajectory
 * @param coords - output coordinates
 * @param coords_dot - output first derivative of coordinates
 * @param coords_dot_dot - output second derivative
 * @param start - vector of start [coordinate, first, second derivative]
 * @param end - vector ot end [coordinate, first, second derivative]
 * @param T - time of maneuver
 * @param steps_count - amount of steps to generate
 * @param dt
 * @return generated steps
 */
int JMT::generate_points_change_coord(
  std::vector<double>& coords,
  std::vector<double>& coords_dot,
  std::vector<double>& coords_dot_dot,
  std::vector<double> start,
  std::vector<double> end,
  double T,
  int steps_count,
  double dt
){
  std::vector<double> trajectory_params;
  generate_parameters(trajectory_params, start, end, T);

  return generate_points(
    coords,
    coords_dot,
    coords_dot_dot,
    trajectory_params,
    T,
    dt,
    steps_count
  );
}
