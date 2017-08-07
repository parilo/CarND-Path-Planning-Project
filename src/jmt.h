#ifndef JMT_H
#define JMT_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

/**
 * @brief class for generation Jerk Minimizing Trajectories (JMT)
 */
class JMT {
 public:

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
  int generate_points_accelerate(
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
  );

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
  int generate_points_change_coord(
    std::vector<double>& coords,
    std::vector<double>& coords_dot,
    std::vector<double>& coords_dot_dot,
    std::vector<double> start,
    std::vector<double> end,
    double T,
    int steps_count,
    double dt
  );

 private:

};

#endif /* JMT_H */
