#include "orbitPropagator.h"




void cwProp(Eigen::MatrixXd& stateHist, const Eigen::Vector3d& r0, const Eigen::Vector3d& v0, const Eigen::MatrixXd& control, const double& tf, const int& intervals, const TrajParams& p){
  Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
  state.head(3) = r0.head(3)/p.nu;
  state.tail(3) = v0.head(3)*p.tau/p.nu;
  double dt = tf/(intervals-1);
  double t = 0;

  for(int i=0;i<intervals-1;i++){
      stateHist.col(i) << state;
      Eigen::VectorXd u = control.col(i);
      rungeKutta(state,t,t+dt,dt/4,u,p,cwDeriv);
      t += dt;
  }
  stateHist.col(intervals-1) << state;
  stateHist *= p.nu;
  stateHist.block(3,0,3,intervals) /= p.tau;
}

// Finds value of y for a given x using step size h 
// and initial value y0 at x0. 
// u is the control input over that timestep
void rungeKutta(Eigen::VectorXd& y, const double& t0, const double& tf, const double& dt, const Eigen::VectorXd& u, const TrajParams& p,
                Eigen::VectorXd (*dydt)(const double&, const Eigen::VectorXd&, const Eigen::VectorXd&, const TrajParams&)){
    // Count number of iterations using step size or 
    // step height h 
    int n = (int)((tf - t0) / dt); 
    double t = t0;

    Eigen::VectorXd k1, k2, k3, k4; 
  
    for (int i=1; i<=n; i++) { 
        // Apply Runge Kutta Formulas to find 
        // next value of y 
        k1 = dt*(*dydt)(t, y, u, p); 
        k2 = dt*(*dydt)(t + 0.5*dt, y + 0.5*k1, u, p); 
        k3 = dt*(*dydt)(t + 0.5*dt, y + 0.5*k2, u, p); 
        k4 = dt*(*dydt)(t + dt, y + k3, u, p); 
  
        // Update next value of y 
        y = y + (1.0/6.0)*(k1 + 2*k2 + 2*k3 + k4);
  
        // Update next value of x 
        t += dt;
    } 
} 

Eigen::VectorXd cwDeriv(const double& t, const Eigen::VectorXd& y, const Eigen::VectorXd& u, const TrajParams& p){
    Eigen::VectorXd yDot = Eigen::VectorXd::Zero(y.size());
    
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6,6);
    A << 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1,
       3*p.n*p.n, 0, 0, 0, 2*p.n, 0,
       0, 0, 0, -2*p.n, 0, 0,
       0, 0, -p.n*p.n, 0, 0, 0;
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6,6);
    B.block(3,3,3,3) = Eigen::MatrixXd::Identity(3,3)*( p.F*p.tau*p.tau/p.m/p.nu);

    yDot = A*y + B*u;
    return yDot;
}
