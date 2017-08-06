#ifndef JMT_H
#define JMT_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

/**
 * @brief class for generation Jerk Minimizing Trajectory
 */
class JMT {
 public:

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

//  int generate_points_change_coord(
//    std::vector<double>& coords,
//    std::vector<double>& coords_dot,
//    std::vector<double>& coords_dot_dot,
//    double start,
//    double start_v,
//    double start_a,
//    double end,
//    double end_v,
//    double end_a,
//    double dt,
//    int steps_count
//  );

  int generate_points(
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
