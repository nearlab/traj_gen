#include <Eigen/Dense>
// nh.getParam("Torb", Torb);
// nh.getParam("sc_mass", mass);
// nh.getParam("tau",tau);
// nh.getParam("nu",nu);

class TrajParams{
public:
double dt;//granularity, s
double m;//mass, kg
double F;//Thrust, kg*m/s^2 (N)
double tau;//time constant, s
double nu;//distance constant, m
double mu;//Gravitational Parameter for Earth kg*m^2/s^3
Eigen::Vector3d r;//Orbital radius
double n;//Mean rate
TrajParams(const double& dt, const double& m, const double& F const double& tau, const double& nu, const Eigen::Vector3d& r; const double& mu=3.986e14):dt(dt),m(m),F(F),tau(tau),nu(nu),r(r),mu(mu);



};