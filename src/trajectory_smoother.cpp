#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

#include "trajectory_smoother.h"

void TrajectorySmoother::merge_trajectoies(
  std::vector<double>& out,
  const std::vector<double>& old_trajectory,
  const std::vector<double>& new_trajectory,
  int merge_len
)
{
  merge_len = std::min(int(old_trajectory.size()), merge_len);
  out.resize(new_trajectory.size());
  for(int i=0; i<merge_len; i++)
  {
    out[i] = ((merge_len - i) * old_trajectory[i] + i * new_trajectory[i]) / merge_len;
  }
  std::copy(new_trajectory.begin() + merge_len, new_trajectory.end(), out.begin() + merge_len);
}

void TrajectorySmoother::smooth_trajectory(
  std::vector<double>& trajectory
)
{
  for(int i=1; i<trajectory.size()-1; i++)
  {
    // trajectory[i] = (trajectory[i-1] + trajectory[i] + trajectory[i+1]) / 3.0;
    trajectory[i] = (trajectory[i-1] + trajectory[i+1]) / 2.0;
  }
}
