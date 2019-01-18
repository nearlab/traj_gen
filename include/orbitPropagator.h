#include <Eigen/Dense>
#include "trajParams.h"
#include <ros/ros.h>

#ifndef ORBIT_PROPAGATOR_H
#define ORBIT_PROPAGATOR_H

void cwProp(Eigen::MatrixXd& stateHist, const Eigen::Vector3d& r0, const Eigen::Vector3d& v0, const Eigen::MatrixXd& control, const double& tf, const int& intervals, const TrajParams& p);

// Modified from http://mathfaculty.fullerton.edu/mathews/n2003/rungekuttafehlbergmod.html
// Note that y is modified to provide the answer rather than returning anything
void rungeKutta(Eigen::VectorXd& y, const double& t0, const double& tf, const double& dt, const Eigen::VectorXd& u, const TrajParams& p,
                Eigen::VectorXd (*dydt)(const double&, const Eigen::VectorXd&, const Eigen::VectorXd&, const TrajParams&), int order);

Eigen::VectorXd cwDeriv(const double& t, const Eigen::VectorXd& state, const Eigen::VectorXd& u, const TrajParams& p);






#endif