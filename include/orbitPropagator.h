#include <Eigen/Dense>
#include "trajParams.h"

#ifndef ORBIT_PROPAGATOR_H
#define ORBIT_PROPAGATOR_H

void cwProp(Eigen::MatrixXd& stateHist, const Eigen::Vector3d& r0, const Eigen::Vector3d& v0, const Eigen::MatrixXd& control, const double& tf, const int& intervals, const Trajparams& p);

// Modified from Geeksforgeeks.org
// Note that y is modified to provide the answer rather than returning anything
void rungeKutta(void (*dydt)(double, Eigen::VectorXd), Eigen::VectorXd& y, const double& t0, const double& tf, const double& dt);

Eigen::VectorXd cwDeriv(const double& t, const Eigen::VectorXd& state, const Eigen::VectorXd& u, const TrajParams& p);






#endif