#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdio.h>
#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>

#define PI 3.14159

Eigen::Matrix4d rotationX(const double &alpha);

Eigen::Matrix4d rotationY(const double &alpha);

Eigen::Matrix4d rotationZ(const double &alpha);

//Class: Define properties and methods that the 5-DOF arm
//for Team D in mechatronics uses
class armKinematics{
private:
	Eigen::Matrix4d H1o2i, H2o3i, H3o4i, H4o5i, H5o6i;

  	double x1,x2,x3,x4,x5,x6;
  	double y1,y2,y3,y4,y5,y6;
  	double z1,z2,z3,z4,z5,z6;


public:
	armKinematics();
	void initialize();
	void forwardKinematics(const std::vector<double> &alpha);
	bool inverseKinematics(const geometry_msgs::Pose &pose, std::vector<double> &alpha);
};

#endif