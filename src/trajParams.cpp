#include "trajParams.h"
#include <math.h>

TrajParams::TrajParams(){
  r << 42164000,0,0;
  this.n = sqrt(3.986e14/pow(42164000,3));
}
TrajParams::TrajParams(const double& dt, const double& m, const double& F, const double& tau, const double& nu, const Eigen::Vector3d& r; const double& mu){
  this.n = sqrt(mu/pow(r.norm(),3));
}