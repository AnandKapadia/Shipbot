#include <stdio.h>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <string>
#include "arm_control/arm_traj.h"


class armExecutive{

	arm_control::arm_traj arm_traj_srv;
	ros::NodeHandle n;
	ros::ServiceClient client;

public:
	armExecutive();
 	int initialize();
	int home();
	int approach(geometry_msgs::Pose pose, double distance);
	int move(const geometry_msgs::Pose &pose);
	int manipulate(const geometry_msgs::Pose &pose, bool orientation);
	int depart(geometry_msgs::Pose pose, double distance);
	Eigen::Matrix3d quat2Rot(const geometry_msgs::Pose &pose);
};
armExecutive::armExecutive(){

	client = n.serviceClient<arm_control::arm_traj>("arm_control/arm_traj");
	ros::Duration(1).sleep();

}

//go to home position from current position. does not factor in obstacles in way
int armExecutive::home(){

	ros::Duration(.5).sleep();
  	geometry_msgs::Pose pose;

    pose.orientation.w = 0;
	pose.orientation.x = 1;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.position.x =  -.15;
	pose.position.y = -0.1;
	pose.position.z = 0.0;

	arm_traj_srv.request.endPose = pose;

	if (client.call(arm_traj_srv))
    {
    	std::cout << "task result : " << arm_traj_srv.response.result << std::endl;
  	}
 	else
 	{
    	ROS_ERROR("Failed to call service arm_trajectory");
    	return 1;
    }
    return 0;
}

int armExecutive::approach(geometry_msgs::Pose pose, double distance){
	ros::Duration(.5).sleep();

	//if target horizontal orientation = 1
	//if target vertical, orientation = 0
	Eigen::Matrix3d rotMat = quat2Rot(pose);

	pose.position.x =  pose.position.x - distance*rotMat(2,0);
	pose.position.y =  pose.position.y - distance*rotMat(2,1);
	pose.position.z =  pose.position.z - distance*rotMat(2,2);

	arm_traj_srv.request.endPose = pose;

	if (client.call(arm_traj_srv))
    {
    	std::cout << "task result : " << arm_traj_srv.response.result << std::endl;
  	}
 	else
 	{
    	ROS_ERROR("Failed to call service arm_trajectory");
    	return 1;
    }
    return 0;

}
int armExecutive::move(const geometry_msgs::Pose &pose){
	//ros::Duration(.5).sleep();
		arm_traj_srv.request.endPose = pose;

	if (client.call(arm_traj_srv))
    {
    	std::cout << "task result : " << arm_traj_srv.response.result << std::endl;
  	}
 	else
 	{
    	ROS_ERROR("Failed to call service arm_trajectory");
    	return 1;
    }
    return 0;
}

int armExecutive::depart(geometry_msgs::Pose pose,double distance){
	ros::Duration(.5).sleep();

	//if target horizontal orientation = 1
	//if target vertical, orientation = 0
	Eigen::Matrix3d rotMat = quat2Rot(pose);

	pose.position.x =  pose.position.x - distance*rotMat(2,0);
	pose.position.y =  pose.position.y - distance*rotMat(2,1);
	pose.position.z =  pose.position.z - distance*rotMat(2,2);

	arm_traj_srv.request.endPose = pose;

	if (client.call(arm_traj_srv))
    {
    	std::cout << "task result : " << arm_traj_srv.response.result << std::endl;
  	}
 	else
 	{
    	ROS_ERROR("Failed to call service arm_trajectory");
    	return 1;
    }
    return 0;

}

Eigen::Matrix3d armExecutive::quat2Rot(const geometry_msgs::Pose &pose){
	
	Eigen::Quaternion<double> qu(pose.orientation.w, pose.orientation.x,
						pose.orientation.y, pose.orientation.z);	
	Eigen::Matrix3d rotMat = qu.matrix();

	return rotMat;
}
	
int main(int argc, char **argv)
{
	geometry_msgs::Pose p;
    geometry_msgs::Pose p1;
    p.orientation.w = 0;
	p.orientation.x = 1;
	p.orientation.y = 0;
	p.orientation.z = 0;
	p.position.x =  -.35;
	p.position.y = 0.0;
	p.position.z = 0.2;


	p1.orientation.w = .7077;
	p1.orientation.x = 0;
	p1.orientation.y = -.7077;
	p1.orientation.z = 0;
	p1.position.x =  -.3;
	p1.position.y = 0.2;
	p1.position.z = 0.2;




  ros::init(argc, argv, "arm_trajectory_client");

  armExecutive exec;

  exec.home();
  //ros::Duration(1).sleep();
  exec.approach(p,0.2);
 // ros::Duration(1).sleep();
  exec.move(p);
  //ros::Duration(1).sleep();
  exec.depart(p,0.1);
  exec.home();

  ros::Duration(1).sleep();
  exec.approach(p1,0.2);
  ros::Duration(1).sleep();
  exec.move(p1);
  ros::Duration(1).sleep();
  exec.home();



  return 0;
}