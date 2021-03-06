#include "orbitPropagator.h"
#include "params.h"
#include <math.h>
#include <cmath>
//#include <../third_party/NumericalIntegration/NumericalIntegration.h>

#ifndef ENERGY_OPTIMAL_H
#define ENERGY_OPTIMAL_H

class Waypoint{
public:
  Eigen::Vector3d r,v;
  double t;
  Waypoint(const Eigen::Vector3d &r, const Eigen::Vector3d &v, const double &t):r(r),v(v),t(t){}
};
//Given multiple waypoints with first and last having velocities, generate the optimal trajectory
void energyOptimalWps(Eigen::Matrix3Xd control, const Waypoint* wps, const double* times, const int len, const OrbitalParams& p);
//Given multiple waypoints "" "", genereate the optimal interim velocities
void optimalVelocityAlg(Eigen::Matrix3Xd control, const Waypoint* wps, int len, const OrbitalParams& p);
//Given two waypoints with start and end velocities, generate the optimal trajectory.
void energyOptimal(Eigen::MatrixXd& control, const Waypoint& start, const Waypoint& end, const int& intervals, const OrbitalParams& p);

void integrateGramian(Eigen::MatrixXd& W, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const double& t0, const double& tf, const int& intervals);


#endif
