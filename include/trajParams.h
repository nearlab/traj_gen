#include <Eigen/Dense>
#include <math.h>

#ifndef TRAJ_PARAMS_H
#define TRAJ_PARAMS_H
class TrajParams{
public:
double m;//mass, kg
double F;//Thrust, kg*m/s^2 (N)
double tau;//time constant, s
double nu;//distance constant, m
double mu;//Gravitational Parameter for Earth kg*m^2/s^3
Eigen::Vector3d r;//Orbital radius
double n;//Mean rate
TrajParams():m(100),F(.1),tau(100),nu(300),mu(3.986e14){
  this->r << 42164000,0,0;
  this->n = sqrt(3.986e14/pow(42164000,3));
}
TrajParams(const double& m, const double& F, const double& tau, const double& nu, const Eigen::Vector3d& r, const double& mu=3.986e14):m(m),F(F),tau(tau),nu(nu),mu(mu),r(r){
  this->n = sqrt(mu/pow(r.norm(),3));
}



};
#endif