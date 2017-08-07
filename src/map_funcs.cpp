#include <iostream>
#include <math.h>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "map_funcs.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = fabs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

/** Transform from Cartesian x,y coordinates to Frenet s,d coordinates
 * @brief getFrenet
 * @param x
 * @param y
 * @param theta
 * @param maps_x
 * @param maps_y
 * @return vector [s, d]
 */
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

/**
 * @brief Transform from Frenet s,d coordinates to Cartesian x,y
 * @param s
 * @param d
 * @param maps_s
 * @param maps_x
 * @param maps_y
 * @return vector [x, y]
 */
vector<double> getXY(
  double s,
  double d,
  const vector<double>& maps_s,
  const vector<double>& maps_x,
  const vector<double>& maps_y
)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

/**
 * @brief Get maximum curvature of specified length road segment
 * @param s - current s
 * @param d - current d
 * @param length - segment length, begins from s
 * @param maps_s - maps s keypoints
 * @param maps_x - maps x keypoints
 * @param maps_y - maps y keypoints
 * @return
 */
double getMaxCurvatureOfRoad(
  double s,
  double d,
  double length,
  const vector<double>& maps_s,
  const vector<double>& maps_x,
  const vector<double>& maps_y
)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  double curvature = 0;
  for (int i=prev_wp; i<maps_x.size()-2; i++){
    if (maps_s[i] > s + length) break;
    double ds = maps_s[i+1] - maps_s[i]; // delta s
    double xd = (maps_x[i+1] - maps_x[i]) / ds; // first derivative of x
    double xd2 = (maps_x[i+2] - 2 * maps_x[i+1] + maps_x[i]) / ds / ds; // second derivative of x
    double yd = (maps_y[i+1] - maps_y[i]) / ds; // first derivative of y
    double yd2 = (maps_y[i+2] - 2 * maps_y[i+1] + maps_y[i]) / ds / ds; // second derivative of y

    double cur_curvature = fabs(xd * yd2 - yd * xd2) / std::pow(xd*xd + yd*yd, 1.5); // current point road curvature
    if (cur_curvature > curvature) {
      curvature = cur_curvature;
    }
  }

  return curvature;

}

/**
 * @brief Interpolate with splines sparsed maps keyupoints
 * @param s - current s
 * @param splined_maps_s - output interpolated values
 * @param splined_maps_x - output interpolated values
 * @param splined_maps_y - output interpolated values
 * @param maps_s - maps keypoints
 * @param maps_x - maps keypoints
 * @param maps_y - maps keypoints
 */
void getSplinedMapPoints(
  double s,
  std::vector<double>& splined_maps_s,
  std::vector<double>& splined_maps_x,
  std::vector<double>& splined_maps_y,
  const std::vector<double>& maps_s,
  const std::vector<double>& maps_x,
  const std::vector<double>& maps_y
)
{

  int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

  // calculating spline based on 4 behind, current, 4 ahead keypoints
  static const int spline_edge_length = 4;
  static const double track_length = 6945.554;
  int spline_input_points = spline_edge_length * 2 + 1;

  std::vector<double> T(spline_input_points),
    S(spline_input_points),
    X(spline_input_points),
    Y(spline_input_points);

  int maps_points_size = maps_x.size();
  for(int i=0; i<spline_input_points; i++)
  {
    // parameter for splines
    T[i] = i - spline_edge_length;
    int maps_point_outer_index = prev_wp + i - spline_edge_length;
    int maps_point_index = maps_point_outer_index % maps_points_size;
    if (maps_point_index < 0) maps_point_index += maps_points_size;

    X[i] = maps_x[maps_point_index];
    Y[i] = maps_y[maps_point_index];

    // need to handle s jump from 6945.554 to 0
    if (maps_point_outer_index < 0)
    {
      S[i] = maps_s[maps_point_index] - track_length;
    }
    else if (maps_point_outer_index >= maps_points_size)
    {
      S[i] = maps_s[maps_point_index] + track_length;
    }
    else
    {
      S[i] = maps_s[maps_point_index];
    }
  }

  tk::spline s_spline;
  s_spline.set_points(T,S);
  tk::spline x;
  x.set_points(T,X);
  tk::spline y;
  y.set_points(T,Y);

  // interpolated step size
  double t_step = 1.0 / 30.0 / spline_input_points;
  for (
    double t = -double(spline_edge_length);
    t < double(spline_edge_length);
    t += t_step
  ){
    splined_maps_s.push_back (s_spline(t));
    splined_maps_x.push_back (x(t));
    splined_maps_y.push_back (y(t));
  }
}
