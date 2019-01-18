#include <Eigen/Dense>
#include "trajParams.h"

#ifndef ORBIT_PROPAGATOR_H
#define ORBIT_PROPAGATOR_H

void cwProp(Eigen::Vector3d r0, Eigen::Vector3d v0, Eigen::MatrixXd control, double tf, int intervals);

// Modified from Geeksforgeeks.org
// Note that y is modified to provide the answer rather than returning anything
void rungeKutta(void (*dydt)(double, Eigen::VectorXd), Eigen::VectorXd& y, const double& t0, const double& tf, const double& dt);

void cwDeriv(double t, Eigen::VectorXd state, TrajParams p);






#endif