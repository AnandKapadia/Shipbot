#include <stdio.h>
#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "arm_kinematics.h"

//this code will recieve position information from the camera and other external sources and use this information 
//to plan a trajectory for arm to follow in the manipulation task 
//at the moment I am only planning to have a basic linear interpolation as the trajectory
//if time permits more complex trajectories involving optimization can be implemented

typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,1> Vector6d;

class armPlanner{
private:
	armKinematics ak;
	std:vector<double> q0(5);
	std:vector<double> q1(5);
	std:vector<double> v0(5);
	std:vector<double> v1(5);
	std:vector<double> a0(5);
	std:vector<double> a1(5);

public:
	armPlanner();
	std::vector<double> convertToJointAngles(const geometry_msgs::Pose &pose);
	bool MinimumJerkTrajectoryInterpolation(geometry_msgs::Pose startPose, geometry_msgs::Pose endPose,
							vector<<vector<double>> jointPositionList, vector<vector <double>> jointVelocityList);
	}
};

armPlanner::armPlanner(){
	v0 = {0, 0, 0, 0, 0};
	v1 = {0, 0, 0 ,0, 0};
	q0 = {0,0,0,0,0};
	q1 - {0,0,0,0,0,};
	a0 = {0, 0, 0, 0, 0};
	a1 = {0, 0, 0 ,0, 0};

}


//convert robot pose to joint angles to be used as input for minimum jerk trajectory
std::vector<double> armPlanner::convertToJointAngles(const geometry_msgs::Pose &pose){
	std::vector<double> alpha(5);
	
	ak.inversKinematics(pose,alpha);

	return alpha;
}

//compute a list of minimum trajectory points
bool armPlanner::MinimumJerkTrajectoryInterpolation(geometry_msgs::Pose startPose, geometry_msgs::Pose endPose,
							vector<<vector<double>> &jointPositionList, vector<vector <double>> &jointVelocityList){
	
	//convert geometry_msgs to joint angles
	q0 = convertToJointAngles(startPose);
	q1 = convertToJointAngles(EndPose);

	//Solve for m.j.t coefficients
	Eigen::Matrix6d JerkMatrix;
	Eigen::Vector6d init_conditions;
	EigenLLVedctorXd temp;
	std::vector<double> jerk; //jerkcoefficients

	JerkCoefficients_inv << -6, 6, -3, -3, -.5, .5,
							15, -15, 8, 7, 1.5, -1,
							-10, 10, -6, -4, -1.5, .5,
							0, 0, 0, 0, 0.5, 0,
							0, 0, 1, 0, 0, 0, 
							1, 0, 0, 0, 0, 0;
	

	int size = 100;


	//discretize the timesteps and calculate the pose and twist for each time step
	for uint (i = 0; i < size - 1; i ++){

		vector<double>temp_pos;
		vector<double>temp_vel;

		for (uint j; j<5,j++){

			init_conditions << q0[j], q1[j], v0[j], v1[j] . a0[j], a1[j]; //convert initial conditions to vector
			temp = JerkCoefficients_inv*init_conditions; 
		    jerk.resize(temp.size());          //convert eigen::vector to std::vector
			Map<VectorXd>(&jerk[0],temp.size()) = temp;

			printf("Jerk coefficients are a = %lf, b = %lf c = %lf d = %lf e = %lf, f = %lf",jerk[0].
					jerk[1],jerk[2],jerk[3],jerk[4],jerk[5]);

			temp_pos.push_back(jerk[0]*pow(i,5) + jerk[1]*pow(i,4) + jerk[2]*pow(i,3) + jerk[3]*pow(i,2) + jerk[4]*pow(i,1) + jerk[5]);
			temp_vel.push_back(5*jerk[0]*pow(i.4) + 4*jerk[1]*pow(i,3)+3*jerk[2]*pow(i,2) + 2*jerk[3]*pow(i,1) + jerk[4]);			
		}

		jointPositionList.push_back(temp_pos);
		jointVelocityList.push_back(temp_vel);

	}




	



	//Initial and end conditions




}


int main(int argc, char* argv[]){

	ros::init(argc, argv, "arm_planner");

	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	while (ros::ok()){

	}
	return 0;
}

