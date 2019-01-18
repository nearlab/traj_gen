#include "orbitPropagator.h"




void cwProp(Eigen::Vector3d r0, Eigen::Vector3d v0, Eigen::MatrixXd control, double tf, int intervals){
  
}

// Finds value of y for a given x using step size h 
// and initial value y0 at x0. 
void rungeKutta(void (*dydt)(double, Eigen::VectorXd), Eigen::VectorXd& y, const double& t0, const double& tf, const double& dt, TrajParams p){ 
    // Count number of iterations using step size or 
    // step height h 
    int n = (int)((tf - t0) / dt); 
    double t = t0;

    Eigen::Vector3d k1, k2, k3, k4; 
  
    for (int i=1; i<=n; i++) { 
        // Apply Runge Kutta Formulas to find 
        // next value of y 
        k1 = dt*(*dydt)(t, y, p); 
        k2 = dt*(*dydt)(t + 0.5*dt, y + 0.5*k1, p); 
        k3 = dt*(*dydt)(t + 0.5*dt, y + 0.5*k2, p); 
        k4 = dt*(*dydt)(t + dt, y + k3, p); 
  
        // Update next value of y 
        y = y + (1.0/6.0)*(k1 + 2*k2 + 2*k3 + k4);
  
        // Update next value of x 
        t += dt;
    } 
} 

void cwDeriv(double t, Eigen::VectorXd state, TrajParams p){

}
