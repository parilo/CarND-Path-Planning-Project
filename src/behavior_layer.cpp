#include <iostream>
#include "behavior_layer.h"
#include "map_funcs.h"
#include "trajectory_smoother.h"

void BehaviorLayer::set_map_waypoints (
  const std::vector<double>& map_waypoints_s,
  const std::vector<double>& map_waypoints_x,
  const std::vector<double>& map_waypoints_y
)
{
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
}

int get_lane_index (double car_d)
{
  // lanes centers positions in d are: 2.2, 6.2, 10.2 in average
  return int(round((car_d - 2.2) / 4.0));
}

void get_lanes_values (
  std::vector<double>& lanes_values,
  const std::vector<double>& car_state,
  const std::vector<std::vector<double>>& sensor_data
)
{
  lanes_values.resize(3, 1000);
  double car_s = car_state[2];
  double car_s_dot = car_state[5];

  // inspecting information about other cars on the road
  for (auto& other_car_data : sensor_data)
  {
    // car id, x, y, vx, vy, s, d
    double other_car_vx = other_car_data[3];
    double other_car_vy = other_car_data[4];
    double other_car_s = other_car_data[5];
    double other_car_d = other_car_data[6];

    // determine lane of the car
    int lane_index = get_lane_index (other_car_d);
// std::cout << "  other car lane: " << lane_index << std::endl;
    // determine ds/dt, suppose car travels along the road
    double other_car_s_dot = distance (0, 0, other_car_vx, other_car_vy);

    if (other_car_s_dot != car_s_dot) {
      double time_to_collision = (car_s - other_car_s) / (other_car_s_dot - car_s_dot);
// std::cout << "    v: " << time_to_collision << std::endl;
      if (
        time_to_collision >= 0 &&
        lanes_values[lane_index] > time_to_collision
      ) {
        lanes_values[lane_index] = time_to_collision;
      }
    }
  }

// std::cout << "  lanes values: ";
//   for(int i=0; i<3; i++)
//   {
// std::cout << lanes_values[i] << " ";
//   }
// std::cout << std::endl;
}

int BehaviorLayer::try_change_lane (
  const std::vector<double>& car_state,
  const std::vector<double>& lanes_values
)
{
  double car_d = car_state[3];
  int my_lane_index = get_lane_index (car_d);

  if (my_lane_index == 0)
  {
    if (lanes_values[1] > 2 * forward_until_time){
      return 1;
    } else {
      return 0;
    }
  }
  else if (my_lane_index == 1)
  {
    if (lanes_values[0] > lanes_values[2]){
      if (lanes_values[0] > 2 * forward_until_time)
      {
        return -1;
      } else {
        return 0;
      }
    } else {
      if (lanes_values[2] > 2 * forward_until_time)
      {
        return 1;
      } else {
        return 0;
      }
    }
  }
  else if (my_lane_index == 2)
  {
    if (lanes_values[1] > 2 * forward_until_time){
      return -1;
    } else {
      return 0;
    }
  }

  return 0;
}

bool BehaviorLayer::is_dst_lane_reached (const std::vector<double>& car_state)
{
  double car_d = car_state[3];
  double dst_lane_center_d = dst_lane_index * 4 + 2.2;
  return fabs(dst_lane_center_d - car_d) < 0.5;
}

bool BehaviorLayer::update_current_state (
  const std::vector<double>& car_state,
  const std::vector<std::vector<double>>& sensor_data
)
{
  double car_d = car_state[3];

  // need to get lanes_values before decision making
  // lanes values are time to collision for current lane
  std::vector<double> lanes_values;
  get_lanes_values (
    lanes_values,
    car_state,
    sensor_data
  );

  CarState prev_state = current_state;

  switch(current_state){
    case CarState::MOVE_FORWARD:
      int my_lane_index = get_lane_index (car_d);
      if (lanes_values [my_lane_index] < forward_until_time)
      {
        //we need to decide wether we need to:
        //1. change lane
        //2. follow next car in the lane
        int lane_index_change = try_change_lane (car_state, lanes_values);
        switch(lane_index_change)
        {
          case -1:
            current_state = CarState::CHANGING_LEFT;
          break;
          case 0:
            current_state = CarState::FOLLOW;
          break;
          case 1:
            current_state = CarState::CHANGING_RIGHT;
          break;
        }
        dst_lane_index = my_lane_index + lane_index_change;

      }
    break;

    case CarState::CHANGING_LEFT:
    case CarState::CHANGING_RIGHT:
    {
      if (is_dst_lane_reached(car_state)) {
        current_state = CarState::MOVE_FORWARD;
        update_current_state (car_state, sensor_data);
      };
      break;
    }

  //   case CarState::FOLLOW:
  //     int lane_index_change = try_change_lane (car_state, lanes_values);
  //     switch(lane_index_change)
  //     {
  //       case -1:
  //         current_state = CarState::CHANGING_LEFT;
  //       break;
  //       case 1:
  //         current_state = CarState::CHANGING_RIGHT;
  //       break;
  //     }
  //     dst_lane_index = my_lane_index + lane_index_change;
  //   break;
  }

  return prev_state != current_state;
}

void BehaviorLayer::process_step (
  std::vector<double>& next_x_vals,
  std::vector<double>& next_y_vals,
  const std::vector<double>& car_state, // x, y, s, d, yaw, v
  const std::vector<double>& previous_path_x,
  const std::vector<double>& previous_path_y,
  const std::vector<std::vector<double>>& sensor_data
)
{

  bool state_changed = update_current_state (car_state, sensor_data);
  std::cout << "car state: " << int(current_state) << std::endl;

  double car_s = car_state[2];
  double car_d = car_state[3];

  int passed_steps = maneuver_planner.get_steps_left() - previous_path_x.size();
  if (previous_path_x.size() < 150) {

    std::vector<double> next_s_vals;
    maneuver_planner.init_maneuver(car_s);
    maneuver_planner.get_next_coords(next_s_vals);

    double px=0, py=0;
    std::vector<double> new_xs, new_ys;
    for (int i=0; i<next_s_vals.size(); i++) {
      std::vector<double> xy = getXYSplined(next_s_vals[i], car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      new_xs.push_back(xy[0]);
      new_ys.push_back(xy[1]);
    }

    TrajectorySmoother::merge_trajectoies(next_x_vals, previous_path_x, new_xs, 100);
    TrajectorySmoother::smooth_trajectory(next_x_vals);
    TrajectorySmoother::smooth_trajectory(next_x_vals);
    TrajectorySmoother::merge_trajectoies(next_y_vals, previous_path_y, new_ys, 100);
    TrajectorySmoother::smooth_trajectory(next_y_vals);
    TrajectorySmoother::smooth_trajectory(next_y_vals);
  } else {
    maneuver_planner.update_maneuver(
      passed_steps,
      car_s
    );
    next_x_vals.resize(previous_path_x.size());
    next_y_vals.resize(previous_path_y.size());
    copy(previous_path_x.begin(), previous_path_x.end(), next_x_vals.begin());
    copy(previous_path_y.begin(), previous_path_y.end(), next_y_vals.begin());
  }

}
