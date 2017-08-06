# Highway driving car autopilot planner

[![Highway driving car autopilot planner demo](http://img.youtube.com/vi/v4lRn_nQXrE/0.jpg)](https://www.youtube.com/watch?v=v4lRn_nQXrE)

This is project 1 of Term 3 of Udacity Self-Driving Car Nanodegree. The goal of the project is to plan maneuvers on highway in randomized environment. Environment contains other cars. Autopilot needs to be able to drive entire loop of 4.32 miles without experience total acceleration over 10 m/s^2, jerk that is greater than 10 m/s^3, other cars collisions and speed limit of 50 mph violations.

## Overview

This highway planner which purpose is produce smooth trajectories for acceleration, deceleration, change lane and front car following maneuvers along with making decisions in changing highway environment contain following modules:

- Trajectory generator
- Maneuver planner
- Localization module
- Behavioral layer

## Trajectory generator

Used trajectory generator is based on [jerk minimizing trajectory (JMT)](http://www.shadmehrlab.org/book/minimum_jerk/minimumjerk.htm). Which represents as following formulation:

```
s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
a_x - parameters of the trajectory
```

Dealing with JMT in common is not an easy thing. But if your task to calculate parameters of JMT for given start coordinate, start velocity, start acceleration, end desired coordinate, desired velocity, desired acceleration and T - maneuver time the problem have well known solution. And I use it for other maneuvers calculations.

For construction 2d highway maneuvers like acceleration and change line we need to solve two different 1d problems with JMT. First is acceleration trajectory which begins from start coordinate, start velocity, start acceleration and ends with some desired velocity and acceleration. Note that we don't know T - time of whole maneuver and end coordinate S. We need to choose S and T in order to have  trajectory with monotonely changing speed. To acheve this I've written simple optimizer that tries to brouteforce S and T in order to get monotonely changing speed trajectory. Important thing is that we need to start optimization from some point close to solution. I have separated start points for acceleration and deceleration.

```
  double T = fabs(end_v - start_v) / 5.0;

  // acceleration
  double S = 3 * end_v * T;

  // deceleration
  double S = start_v * T;
```

Optimizer fixes T and brouteforce S. On every step it check wether trajectory exceeds desired velocity or have velocity turn and tune S accordingly. Optimizer ends its work on trajectory without mentioned drawbacks or if optimization step becomes very small. Code of optimizer looks like this:

```
  double S = end_0; // starting value of S
  int S_mod = 0; // S modification direction
  int prev_S_mod = S_mod;
  double S_step = 0.2; // step size of S modification
  do {
    S += S_mod * S_step;
    jmt_generation_function(params, {start, start_v, start_a}, {start + S, end_v, end_a}, T);
    S_mod = trajectory_check_function(end_v, T, dt, params);

    if (prev_S_mod != S_mod && prev_S_mod != 0) {
      S_step *= 0.5;
    }
    prev_S_mod = S_mod;

  } while (S_mod != 0 && S_step > 1e-3);

  // see jmt.cpp
```

Second 1d trajectory we need is changing coordinate trajectory. It helps with keep and change lane trajectories. Here we know end coordinate and we only need to reasonably choose T - maneuver time for example based on coordinate change distance. So here we just may use JMT parameters generation function.

## Maneuver planner

Using 1d trajectories we may construct 2d maneuvers like

- Acceleration 
   - 1d acceleration along the road
   - lane position adjusting perpendicularly the road
- Change lane
   - keeping velocity along the road
   - changing coordinate perpendicularly the road

```
class ManeuverPlanner1d in maneuver_planner_1d.cpp - helps to construct 1d maneuvers
class ManeuverPlanner in maneuver_planner.cpp - for construction of 2d maneuvers
```

## Localization module

All maneuvers calculations are done in [Frenet frame](https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas). Frenet frame for curved road is coordinates (s, d). 's' is coordinate along the road and d is perpendiculary coordinate. After maneuver calculation in (s, d) we need to translate it into world space (x, y). We have "getXY" function in map\_funcs.cpp for it. It receives (s, d) coordinates and its local keypoints (number of corresponding x, y, s) and provides (x, y) of maneuver point. The map of the highway keypoints is in data/highway_map.txt. Each keypoint in the list contains  \[x,y,s,dx,dy\] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554. Text file keypoint are very sparsed and using them as is leads to broken curves. In order to fix broken trajectories we need to interpolate smooth keypoints curve between map's keypoints and use it for (s, d) -> (x, y) conversion. Such interpolation is done with [splines](http://kluge.in-chemnitz.de/opensource/spline/) in "getSplinedMapPoints" function in map\_funcs.cpp.

Frenet frame is not inertial frame for a road. So we also need to calculate future curvature of the road to slow the car on the curves to decrease centrifugal acceleration. See "getMaxCurvatureOfRoad" function in map\_funcs.cpp.


##Behavioral layer

To deal with other cars on the road we need to make decisions when to change lane (and which lane) and pass or follow the front car. The goal is not to stuck behind other cars and avoid collisions. For implementing such logic I use [finite state machine (FSM)](https://en.wikipedia.org/wiki/Finite-state_machine) with such states:

- Start
- Move forward
- Follow front car
- Changing lane to left
- Changing lane to right

States transitions may performs as follows

_ | Start | Forward | Follow | Change Left | Change Right
--- | --- | --- | --- | --- | ---
Start          | _ | + | _ | _ | _
Forward        | _ | _ | + | + | +
Follow         | _ | + | _ | + | +
Change Left    | _ | + | _ | _ | _
Change Right   | _ | + | _ | _ | _

Desisions about to start transitions make using 3 cost functions:

- time to collision on each lane
- distance to front car for each lane
- speed of front car for each lane
- closed lane flag (if other car blocks lane for changing into)

All cost functions are used separately with handcrafted rules. See behavior_layer.cpp for detalis. Behavioral layer monitors information about other cars on every step. And change states accordingly. Current performing maneuver recalculates every 10 steps or on state chage. Also road curvature is measured every 10 steps and allowed forward velocity is adjusted if needed. Other tunable main parameters are:

```
// need to slow avoiding collisions
const double safety_time_to_collision = 10; // secs

// needed space between cars for lane changing
const double safety_change_lane_gap = 10; // meters

// distance of the front car for following
const double safety_front_car_dist = 25; // meters

// future trajectory horizon params
const int maneuver_min_steps_count = 150; // one step is 0.02 sec
const int maneuver_recalc_steps_count = 140;

//maximum forward speed
double forward_speed = 0.44704 * 46; // m/s
```

After calculation of new trajectory we need to properly translate car from old trajectory to new. So as car drives in parallel to new maneuver calculation we need to keep small amount of first steps (10) from an old trajectory and linearly transite to new trajectory. Also we need to smooth new maneuver before application. See smoothing functions in trajectory_smoother.cpp. For other detalis please see code and copmments inside

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Simulator details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

### Simulator.

You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

