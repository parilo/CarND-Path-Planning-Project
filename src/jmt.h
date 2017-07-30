#ifndef JMT_H
#define JMT_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

/**
 * @brief class for generation Jerk Minimizing Trajectory
 */
class JMT {
 public:

  void generate_parameters(
    std::vector<double>& params,
    std::vector<double> start,
    std::vector <double> end,
    double T
  );

  void generate_points(
    std::vector<double>& points,
    std::vector<double> start,
    std::vector <double> end,
    double T,
    int steps_count,
    double dt
  );

 private:

};

#endif /* JMT_H */
