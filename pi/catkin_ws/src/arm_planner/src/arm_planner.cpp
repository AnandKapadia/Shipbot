#include <stdio.h>
#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"


int main(int argc, char* argv[]){

	ros::init(argc, argv, "arm_planner");

	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	while (ros::ok()){

	}
	return 0;
}

