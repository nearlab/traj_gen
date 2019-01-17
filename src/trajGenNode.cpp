#include <ros/ros.h>

#include <Eigen/Dense>

#include "trajParams.h"
#include "energyOptimal.h"
#include "nearlab_msgs/energy_optimal_traj.h"
#include "nearlab_msgs/attitude_traj.h"

bool callbackEnergyOptimalTraj(nearlab_msgs::energy_optimal_traj::Request& req, nearlab_msgs::energy_optimal_traj::Response& res){
  // Make TrajParams object
  int intervals = req.intervals;
  Eigen::Vector3d rOrb, rStart, rEnd, vStart, vEnd;
  for(int i=0;i<3;i++){
    rOrb(i) = req.rOrb[i];
    rStart(i) = req.rStart[i];
    rEnd(i) = req.rEnd[i];
    vStart(i) = req.vStart[i];
    vEnd(i) = req.vEnd[i];
  }
  ROS_INFO("Created vectors");
  TrajParams params(req.sc_mass,req.sc_thrust,req.time_const,req.dist_const,rOrb,req.grav_param);
  ROS_INFO("Created Params");
  // Other stuff
  Waypoint wpStart(rStart,vStart,req.tStart);
  Waypoint wpEnd(rEnd,vEnd,req.tEnd);
  Eigen::MatrixXd control;
  ROS_INFO("Starting trajectory generation");
  energyOptimal(control,wpStart,wpEnd,intervals,params);
  ROS_INFO("Finished trajectory generation");
  double dt = (req.tEnd-req.tStart)/(intervals-1);

  for(int i=0;i<intervals;i++){
    res.control_x.push_back(control(3,i));
    res.control_y.push_back(control(4,i));
    res.control_z.push_back(control(5,i));
    res.times.push_back(dt*i);
  }
  return true;
}

// For attitude: Look up SLURP method (vary angle from initial to final given values 0 - 1) 
  //               Perhaps vary 0 to 1 nonlinearly in time to get smooth tr
bool callbackAttitudeTraj(nearlab_msgs::attitude_traj::Request& req, nearlab_msgs::attitude_traj::Response& res){
//   theta = acos(dq(4));
// n = dq(1:3)/norm(dq(1:3));
// dqHalf = [n*sin(.5*theta);cos(.5*theta)];
// qProp = quatMath.quatRot(qOld,dqHalf);
  return false;
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
  ros::ServiceServer energyOptimalServer = nh.advertiseService("/traj_gen/energy_optimal_traj",callbackEnergyOptimalTraj);
  ROS_INFO("Ready to generate trajectories.");
  ros::spin();
}