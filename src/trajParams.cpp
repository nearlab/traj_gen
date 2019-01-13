#include "trajParams.h"
#include <math.h>

TrajParams::TrajParams(const double& dt, const double& m, const double& F, const double& tau, const double& nu, const Eigen::Vector3d& r; const double& mu){
  this.n = sqrt(mu/pow(r.norm(),3));
}