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

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
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

// Transform from Frenet s,d coordinates to Cartesian x,y
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

  std::cout << "splined: car_s: " << s << " prev_wp: " << prev_wp << std::endl;

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
    T[i] = i - spline_edge_length;
    int maps_point_outer_index = prev_wp + i - spline_edge_length;
    int maps_point_index = maps_point_outer_index % maps_points_size;
    if (maps_point_index < 0) maps_point_index += maps_points_size;

    X[i] = maps_x[maps_point_index];
    Y[i] = maps_y[maps_point_index];

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

  std::cout << "T: ";
  for(int i=0; i<spline_input_points; i++){
    std::cout << T[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "S: ";
  for(int i=0; i<spline_input_points; i++){
    std::cout << S[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "X: ";
  for(int i=0; i<spline_input_points; i++){
    std::cout << X[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "Y: ";
  for(int i=0; i<spline_input_points; i++){
    std::cout << Y[i] << " ";
  }
  std::cout << std::endl;

//  int prev_wp4 = (prev_wp-3)%maps_x.size();
//  int prev_wp3 = (prev_wp-2)%maps_x.size();
//  int prev_wp2 = (prev_wp-1)%maps_x.size();
//	int wp2 = (prev_wp+1)%maps_x.size();
//  int wp3 = (prev_wp+2)%maps_x.size();
//  int wp4 = (prev_wp+3)%maps_x.size();

//  T[0]=-3; T[1]=-2; T[2]=-1; T[3]=0; T[4]=1; T[5]=2; T[6]=3;
//  S[0]=maps_s[prev_wp4]; S[1]=maps_s[prev_wp3]; S[2]=maps_s[prev_wp2]; S[3]=maps_s[prev_wp]; S[4]=maps_s[wp2]; S[5]=maps_s[wp3]; S[6]=maps_s[wp4];
//  X[0]=maps_x[prev_wp4]; X[1]=maps_x[prev_wp3]; X[2]=maps_x[prev_wp2]; X[3]=maps_x[prev_wp]; X[4]=maps_x[wp2]; X[5]=maps_x[wp3]; X[6]=maps_x[wp4];
//  Y[0]=maps_y[prev_wp4]; Y[1]=maps_y[prev_wp3]; Y[2]=maps_y[prev_wp2]; Y[3]=maps_y[prev_wp]; Y[4]=maps_y[wp2]; Y[5]=maps_y[wp3]; Y[6]=maps_y[wp4];

  tk::spline s_spline;
  s_spline.set_points(T,S);
  tk::spline x;
  x.set_points(T,X);
  tk::spline y;
  y.set_points(T,Y);

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

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

vector<double> getXYPolyfit(
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

  int approx_edges = 3;
  int num_points = approx_edges * 2 + 1;
  Eigen::VectorXd T(num_points), S(num_points), X(num_points), Y(num_points);
  for(
    int i=prev_wp - approx_edges, vi = 0;
    i<(prev_wp + approx_edges);
    i++, vi++
  ){
    int pi = (i)%maps_x.size();
    T[vi] = i;
    S[vi] = maps_s[pi];
    X[vi] = maps_x[pi];
    Y[vi] = maps_y[pi];
  }

  auto coeffs_s = polyfit(T, S, 3);
  auto coeffs_x = polyfit(T, X, 3);
  auto coeffs_y = polyfit(T, Y, 3);

  std::vector<double> approx_maps_s;
  std::vector<double> approx_maps_x;
  std::vector<double> approx_maps_y;

  double t_step = 1 / 30.0;
  for (
    double t = -1;
    t < 1;
    t += t_step
  ){
    approx_maps_s.push_back (polyeval(coeffs_s, t));
    approx_maps_x.push_back (polyeval(coeffs_x, t));
    approx_maps_y.push_back (polyeval(coeffs_y, t));
  }

  // int prev_wp2 = (prev_wp-1)%maps_x.size();
	// int wp2 = (prev_wp+1)%maps_x.size();

  // std::vector<double> T(3), S(3), X(3), Y(3);
  // T[0]=-1; T[1]=0; T[2]=1;
  // S[0]=maps_s[prev_wp2]; S[1]=maps_s[prev_wp]; S[2]=maps_s[wp2];
  // X[0]=maps_x[prev_wp2]; X[1]=maps_x[prev_wp]; X[2]=maps_x[wp2];
  // Y[0]=maps_y[prev_wp2]; Y[1]=maps_y[prev_wp]; Y[2]=maps_y[wp2];

  // Eigen::VectorXd ptsx_car_frame (ptsx.size());
  // Eigen::VectorXd ptsy_car_frame (ptsy.size());
  // for (int i=0; i<ptsx.size(); i++) {
  //   double x = ptsx[i] - px;
  //   double y = ptsy[i] - py;
  //   ptsx_car_frame [i] = x * cos (psi) + y * sin (psi);
  //   ptsy_car_frame [i] = - x * sin (psi) + y * cos (psi);
  // }
  //
  // auto coeffs = polyfit(ptsx_car_frame, ptsy_car_frame, 3);

   return getXY(s, d, approx_maps_s, approx_maps_x, approx_maps_y);
}
