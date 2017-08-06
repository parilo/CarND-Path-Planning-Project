#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

#include "trajectory_smoother.h"

void TrajectorySmoother::merge_trajectoies(
  std::vector<double>& out,
  const std::vector<double>& old_trajectory,
  const std::vector<double>& new_trajectory_,
  int merge_len
)
{
  if (old_trajectory.size()>0){

    std::vector<double> new_trajectory(new_trajectory_.size()-1);
    std::copy(new_trajectory_.begin()+1, new_trajectory_.end(), new_trajectory.begin());
    merge_len = std::min(int(old_trajectory.size()), merge_len);

    out.resize(new_trajectory.size());
    std::copy(old_trajectory.begin(), old_trajectory.begin()+10, out.begin());

    for(int i=10; i<(merge_len+10); i++)
    {
        out[i] = ((merge_len - i + 10) * old_trajectory[i] + (i-10) * new_trajectory[i]) / merge_len;
  //std::cout << "--- merge 6" << std::endl;
  //    } else {
  //      out[i] = old_trajectory[i];
  //    }
    }
    std::copy(new_trajectory.begin() + merge_len, new_trajectory.end(), out.begin() + merge_len);

//    for(int i=0; i<new_trajectory.size(); i++){
//      double oldv = i<old_trajectory.size()?old_trajectory[i]:0;
//      std::cout <<
//        "   old: " << oldv << " " <<
//        " new: " << new_trajectory[i] << " " <<
//        " out: " << out[i] <<
//        std::endl;
//    }

  } else {
    out.resize(new_trajectory_.size());
    std::copy(new_trajectory_.begin(), new_trajectory_.end(), out.begin());
  }
}

//void TrajectorySmoother::merge_trajectoies(
//  std::vector<double>& out,
//  const std::vector<double>& old_trajectory,
//  const std::vector<double>& new_trajectory,
//  int merge_begin,
//  int merge_len
//)
//{
//  merge_len = std::min(int(old_trajectory.size()), merge_len);
//  out.resize(new_trajectory.size());
//  if(old_trajectory.size() > merge_begin){
//    std::copy(old_trajectory.begin(), old_trajectory.begin() + merge_begin, out.begin());
//  }
//  for(int i=0; i<merge_len && i<new_trajectory.size(); i++)
//  {
//    out[i+] = ((merge_len - i) * old_trajectory[i] + i * new_trajectory[i]) / merge_len;
//  }
//  std::copy(new_trajectory.begin() + merge_len, new_trajectory.end(), out.begin() + merge_len);
//}

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
