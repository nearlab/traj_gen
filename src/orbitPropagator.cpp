#include "orbitPropagator.h"
#include <math.h>



void cwProp(Eigen::MatrixXd& stateHist, const Eigen::Vector3d& r0, const Eigen::Vector3d& v0, const Eigen::MatrixXd& control, const double& tf, const int& intervals, const TrajParams& p){
  Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
  state.head(3) = r0.head(3)/p.nu;
  state.tail(3) = v0.head(3)*p.tau/p.nu;
  double dt = tf/(intervals-1)/100;
  double dtConst = dt*100;
  double t = 0;
  double err = 1e-7;
  int iter = 0;
  bool writeNext = true;
  double s = 1;
  while(iter<intervals){
      Eigen::VectorXd u = control.col(iter);
      Eigen::VectorXd state4 = state;
      Eigen::VectorXd state5 = state;
      rungeKutta(state4,t,t+dt,dt,u,p,cwDeriv,4);
      rungeKutta(state5,t,t+dt,dt,u,p,cwDeriv,5);
      
     
      ROS_INFO_STREAM("Time: " << t << "\tdelta-t: "<< dt <<"\tDifference: " << (state5-state4).norm() <<"\nState4\n"<<state4<<"\nState5\n"<<state5);
      
 
      if(writeNext){
        stateHist.col(iter++) << state5;
        writeNext = false;
        
      }else{
        s = pow(err*dt/2/(state5-state4).norm(),.25); 
      }
      if(std::isinf(s)){
        s = 1.2;
      } 
      
      if((state5-state4).cwiseAbs().minCoeff()>err){
        dt = s*dt;
        continue;
      }
      state = state5;
      t += dt;
      if(t>(iter+1)*dtConst){
        
        t = (iter+1)*dtConst;
	      writeNext = true;
      }
      dt = s*dt;
  }
  stateHist.col(intervals-1) << state;
  stateHist *= p.nu;
  stateHist.block(3,0,3,intervals) /= p.tau;
}

// Finds value of y for a given t using step size dt
// and initial value y at t0. 
// u is the control input over that timestep
void rungeKutta(Eigen::VectorXd& y, const double& t0, const double& tf, const double& dt, const Eigen::VectorXd& u, const TrajParams& p,
                Eigen::VectorXd (*dydt)(const double&, const Eigen::VectorXd&, const Eigen::VectorXd&, const TrajParams&), int order){
    // Count number of iterations using step size or 
    // step height h 
    int n = (int)((tf - t0) / dt); 
    double t = t0;

    Eigen::VectorXd k1, k2, k3, k4, k5, k6; 
  
    for (int i=1; i<=n; i++) { 
        // Apply Runge Kutta Formulas to find 
        // next value of y 
        k1 = dt*(*dydt)(t, y, u, p); 
        k2 = dt*(*dydt)(t + 1.0/4*dt, y + 1.0/4*k1, u, p); 
        k3 = dt*(*dydt)(t + 3.0/8*dt, y + 3.0/32*k1 +9/32*k2, u, p); 
        k4 = dt*(*dydt)(t + 12.0/13*dt, y + 1932.0/2197*k1-7200.0/2197*k2+7296.0/2197*k3, u, p); 
        k5 = dt*(*dydt)(t + dt, y +439.0/216*k1-8*k2+3680.0/513*k3-845.0/4104*k4, u, p); 
        k6 = dt*(*dydt)(t + 1.0/2*dt, y -8.0/27*k1+2*k2-3544.0/2565*k3+1859.0/4104*k4-11.0/40*k5, u, p); 
  
        // Update next value of y
        if(order == 4){
            y = y + 25.0/216*k1 + 1408.0/2565*k3 + 2197.0/4104*k4 - 1.0/5*k5;
        }else{
            y = y + 16.0/135*k1 + 6656.0/12825*k3 + 28561.0/56430*k4 -9.0/50*k5 + 2.0/55*k6;
        }
  
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

    Eigen::VectorXd v = u;
    v(3) = -v(3);
    yDot = A*y + B*v;
    return yDot;
}
