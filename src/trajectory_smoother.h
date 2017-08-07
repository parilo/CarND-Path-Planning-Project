#ifndef TRAJECTORY_SMOOTHER_H
#define TRAJECTORY_SMOOTHER_H

#include <vector>

/**
 * @brief Trajectory smoother
 */
class TrajectorySmoother {
 public:

  /**
   * @brief Keep steps of old trajectory, then linear transition to new trajectory
   *   then new trajectory points
   * @param out - output merged trajectory
   * @param old_trajectory
   * @param new_trajectory
   * @param keep_old_steps - number steps to keep from old trajectory
   * @param merge_len - number of steps for linear merge
   */
  static void merge_trajectoies(
    std::vector<double>& out,
    const std::vector<double>& old_trajectory,
    const std::vector<double>& new_trajectory,
    int keep_old_steps,
    int merge_len
  );

  /**
   * @brief smooth trajectory by neighbours filter
   * @param trajectory
   */
  static void smooth_trajectory(
    std::vector<double>& trajectory
  );

 private:

};

#endif /* TRAJECTORY_SMOOTHER_H */
