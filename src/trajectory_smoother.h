#ifndef TRAJECTORY_SMOOTHER_H
#define TRAJECTORY_SMOOTHER_H

#include <vector>

/**
 * @brief Trajectory smoother
 */
class TrajectorySmoother {
 public:

  static void merge_trajectoies(
    std::vector<double>& out,
    const std::vector<double>& old_trajectory,
    const std::vector<double>& new_trajectory,
    int merge_len
  );

  static void smooth_trajectory(
    std::vector<double>& trajectory
  );

 private:

};

#endif /* TRAJECTORY_SMOOTHER_H */
