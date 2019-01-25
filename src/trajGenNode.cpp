#include <ros/ros.h>

#include <Eigen/Dense>

#include "energyOptimal.h"
#include "orbitPropagator.h"
#include "params.h"
#include "nearlab_msgs/energy_optimal_traj.h"
#include "nearlab_msgs/attitude_traj.h"
#include "quatMath.h"

bool callbackEnergyOptimalTraj(nearlab_msgs::energy_optimal_traj::Request& req, nearlab_msgs::energy_optimal_traj::Response& res){
  // Make TrajParams object
  int intervals = req.intervals;
  Eigen::Vector3d rOrb, rStart, rEnd, vStart, vEnd;
  rOrb = Eigen::VectorXd::Zero(3);
  rStart = Eigen::VectorXd::Zero(3);
  rEnd = Eigen::VectorXd::Zero(3);
  vStart = Eigen::VectorXd::Zero(3);
  vEnd = Eigen::VectorXd::Zero(3);
  for(int i=0;i<3;i++){
    rOrb(i) = req.rOrb[i];
    rStart(i) = req.rStart[i];
    rEnd(i) = req.rEnd[i];
    vStart(i) = req.vStart[i];
    vEnd(i) = req.vEnd[i];
  }
  ROS_INFO("Created vectors");
  OrbitalParams params(req.sc_mass,req.sc_thrust,req.time_const,req.dist_const,rOrb,req.grav_param);
  ROS_INFO("Created Params");
  // Other stuff
  Waypoint wpStart(rStart,vStart,req.tStart);
  Waypoint wpEnd(rEnd,vEnd,req.tEnd);

  Eigen::MatrixXd control = Eigen::MatrixXd::Zero(6,intervals);

  ROS_INFO("Starting trajectory generation");
  energyOptimal(control,wpStart,wpEnd,intervals,params);
  ROS_INFO("Finished trajectory generation");

  Eigen::MatrixXd stateHist = Eigen::MatrixXd::Zero(6,intervals+1);

  ROS_INFO("Starting trajectory propagation");
  cwProp(stateHist,rStart,vStart,control,req.tEnd,intervals,params);
  ROS_INFO("Finished trajectory propagation");
  ROS_INFO_STREAM("FINAL STATE: "<<stateHist(0,intervals)<<", "<<stateHist(1,intervals));
  double dt = (req.tEnd-req.tStart)/intervals;
  for(int i=0;i<intervals;i++){
    res.rx.push_back(stateHist(0,i));
    res.ry.push_back(stateHist(1,i));
    res.rz.push_back(stateHist(2,i));
    res.vx.push_back(stateHist(3,i));
    res.vy.push_back(stateHist(4,i));
    res.vz.push_back(stateHist(5,i));
    // res.ux.push_back(control(3,i));
    // res.uy.push_back(control(4,i));
    // res.uz.push_back(control(5,i));
    res.times.push_back(dt*i);
  }
  return true;
}

// For attitude: Look up SLURP method (vary angle from initial to final given values 0 - 1) 
  //               Perhaps vary 0 to 1 nonlinearly in time to get smooth tr
bool callbackAttitudeTraj(nearlab_msgs::attitude_traj::Request& req, nearlab_msgs::attitude_traj::Response& res){
  int intervals = req.intervals;
  Eigen::Vector4d q0,q1,dq;
  q0 << req.qStart[0],req.qStart[1],req.qStart[2],req.qStart[3];
  q1 << req.qEnd[0],req.qEnd[1],req.qEnd[2],req.qEnd[3];
  dq = quatRot(inverse(q0),q1);

  for(int i=0;i<intervals;i++){
    double ratio = ((double)i)/intervals;
    double theta = acos(dq(3));
    Eigen::Vector3d normal = dq.head(3).normalized();
    Eigen::Vector4d dqPartial = Eigen::VectorXd::Zero(4);
    dqPartial.head(3) << normal*sin(ratio*theta);
    dqPartial(3) = cos(ratio*theta);
    Eigen::VectorXd qProp = quatRot(q0,dqPartial);
    res.qx.push_back(qProp(0));
    res.qy.push_back(qProp(1));
    res.qz.push_back(qProp(2));
    res.qw.push_back(qProp(3));
  }
  return true;
}

int main(int argc, char** argv){
  ros::init(argc,argv,"traj_gen");
  ros::NodeHandle nh;
  
  // Advertise service
  ros::ServiceServer energyOptimalServer = nh.advertiseService("/orbot/space/energy_optimal_traj",callbackEnergyOptimalTraj);
  ros::ServiceServer attitudeServer = nh.advertiseService("/orbot/space/attitude_traj",callbackAttitudeTraj);
  ROS_INFO("Ready to generate trajectories.");
  ros::spin();
}
