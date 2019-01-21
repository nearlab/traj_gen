#include "energyOptimal.h"
#include <unsupported/Eigen/MatrixFunctions>

void energyOptimalWps(Eigen::Matrix3Xd& control, const Waypoint* wps, const double* times, const int len, const OrbitalParams& p){

}
//Given multiple waypoints "" "", genereate the optimal interim velocities
void optimalVelocityAlg(Eigen::Matrix3Xd& control, const Waypoint* wps, const int len, const OrbitalParams& p){

}
//Given two waypoints with start and end velocities, generate the optimal trajectory.
void energyOptimal(Eigen::MatrixXd& control, const Waypoint& start, const Waypoint& end, const int& intervals, const OrbitalParams& p){
  double t0 = start.t;
  double tf = end.t;
  double tspan = tf-t0;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6,6);
  A << 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1,
       3*p.n*p.n, 0, 0, 0, 2*p.n, 0,
       0, 0, 0, -2*p.n, 0, 0,
       0, 0, -p.n*p.n, 0, 0, 0;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6,6);
  B.block(3,3,3,3) = Eigen::MatrixXd::Identity(3,3)*( p.F*p.tau*p.tau/p.m/p.nu);
  
  double dt = tspan/intervals;

  Eigen::MatrixXd W = Eigen::MatrixXd::Identity(6,6); //Controllability Gramian
  integrateGramian(W,A,B,t0,tf,1000);
  Eigen::MatrixXd Winv = W.inverse();

  Eigen::MatrixXd stm = (A*dt).exp();
  Eigen::MatrixXd stm_tf = (A*tspan).exp();
  Eigen::MatrixXd stm_i = Eigen::MatrixXd::Identity(stm.rows(),stm.cols());
  Eigen::MatrixXd sum = Eigen::MatrixXd::Zero(stm.rows(),stm.cols());

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
  x0.head(3) << start.r;
  x0.tail(3) << start.v;
  Eigen::VectorXd xf = Eigen::VectorXd::Zero(6);
  xf.head(3) << end.r;
  xf.tail(3) << end.v;

  for(int i=0;i<intervals;i++){ //Quadrature, I'm lazy
    stm_i *= stm; 
    control.block(0,i,6,1) << B.transpose()*stm_i.transpose()*Winv*(stm_tf*xf - x0);
  }
}

void integrateGramian(Eigen::MatrixXd& W, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const double& t0, const double& tf, const int& intervals){
  double dt = (tf-t0)/(intervals-1);
  Eigen::MatrixXd stm = (A*dt).exp();
  Eigen::MatrixXd stm_i = Eigen::MatrixXd::Identity(stm.rows(),stm.cols());
  Eigen::MatrixXd sum = Eigen::MatrixXd::Zero(stm.rows(),stm.cols());
  for(int i=0;i<intervals;i++){ //Quadrature, I'm lazy
    Eigen::MatrixXd rect;
    stm_i *= stm; 
    sum += stm_i*B*B.transpose()*stm_i.transpose()*dt;
  }
  W << sum;//Return this, essentially
}







