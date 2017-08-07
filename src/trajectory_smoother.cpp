#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

#include "trajectory_smoother.h"

/**
 * @brief Keep 10 steps of old trajectory, then linear transition to new trajectory
 *   then new trajectory points
 * @param out - output merged trajectory
 * @param old_trajectory
 * @param new_trajectory
 * @param keep_old_steps - number steps to keep from old trajectory
 * @param merge_len - number of steps for linear merge
 */
void TrajectorySmoother::merge_trajectoies(
  std::vector<double>& out,
  const std::vector<double>& old_trajectory,
  const std::vector<double>& new_trajectory_,
  int keep_old_steps,
  int merge_len
)
{
  if (old_trajectory.size()>0){

    // removing first point from trajectory since
    // it is current state point while old trajectory holds only future points
    std::vector<double> new_trajectory(new_trajectory_.size()-1);
    std::copy(new_trajectory_.begin()+1, new_trajectory_.end(), new_trajectory.begin());
    merge_len = std::min(int(old_trajectory.size()), merge_len);

    out.resize(new_trajectory.size());
    std::copy(old_trajectory.begin(), old_trajectory.begin()+keep_old_steps, out.begin());

    for(int i=keep_old_steps; i<(merge_len+keep_old_steps); i++)
    {
        out[i] = ((merge_len - i + keep_old_steps) * old_trajectory[i] + (i-keep_old_steps) * new_trajectory[i]) / merge_len;
    }
    std::copy(new_trajectory.begin() + merge_len, new_trajectory.end(), out.begin() + merge_len);

  } else {
    out.resize(new_trajectory_.size());
    std::copy(new_trajectory_.begin(), new_trajectory_.end(), out.begin());
  }
}

/**
 * @brief smooth trajectory by neighbours filter
 * @param trajectory
 */
void TrajectorySmoother::smooth_trajectory(
  std::vector<double>& trajectory
)
{
  for(int i=1; i<trajectory.size()-1; i++)
  {
    trajectory[i] = (trajectory[i-1] + trajectory[i+1]) / 2.0;
  }
}
