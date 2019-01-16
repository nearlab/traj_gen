#include <ros/ros.h>

#include <Eigen/Dense>

#include "trajParams.h"
#include "energyOptimal.h"
#include "nearlab_msgs/energy_optimal_traj.h"

bool callbackEnergyOptimalTraj(nearlab_msgs::energy_optimal_traj::Request& req, nearlab_msgs::energy_optimal_traj::Response& res){
  // Make TrajParams object
  Eigen::Vector3d rOrb, rStart, rEnd, vStart, vEnd;
  for(int i=0;i<3;i++){
    rOrb(i) = req.rOrb[i];
    rStart(i) = req.rStart[i];
    rEnd(i) = req.rEnd[i];
    vStart(i) = req.vStart[i];
    vEnd(i) = req.vEnd[i];
  }

  TrajParams params(req.dt,req.sc_mass,req.sc_thrust,req.time_const,req.dist_const,rOrb,req.grav_param);
  
  // Other stuff
  Waypoint wpStart(rStart,vStart,req.tStart);
  Waypoint wpEnd(rEnd,vEnd,req.tEnd);
  Eigen::MatrixXd control;

  energyOptimal(control,wpStart,wpEnd,params);
  
  for(int i=0;i<control.cols();i++){
    res.control_x.push_back(control(3,i));
    res.control_y.push_back(control(4,i));
    res.control_z.push_back(control(5,i));
  }
  return true;
}

int main(int argc, char** argv){
  ros::init(argc,argv,"traj_gen");
  ros::NodeHandle nh;

  // nh.getParam("dt",dt);
  // nh.getParam("sc_mass", m);
  // nh.getParam("tau",tau);
  // nh.getParam("nu",nu);
  // nh.getParam("sc_thrust", F);
  // nh.getParam("mu",mu);
  // nh.getParam("initial_radius_x",rx);
  // nh.getParam("initial_radius_y",ry);
  // nh.getParam("initial_radius_z",rz);

  // Advertise service
  ros::ServiceServer outputFormatServer = nh.advertiseService("/traj_gen/energy_optimal_traj",callbackEnergyOptimalTraj);
  // 
  ros::spin();
}