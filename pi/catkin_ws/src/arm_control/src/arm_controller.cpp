#include <stdio.h>
#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include "eigen3/Eigen/src/SVD/JacobiSVD.h"
#include <vector>
#include <math.h>
#include "lookup.hpp"
#include "feedback.hpp"
#include "feedback_print_helpers.cpp"
#include "module.hpp"
#include "group.hpp"
#include "command.hpp"
#include "mac_address.hpp"
#include "lookup_helpers.cpp"
#include "hebi_util.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#define PI 3.14159
#define Gravity 9.81


Eigen::Matrix4d rotationX(const double &alpha){
	Eigen::Matrix4d rotation;
	rotation << 1,     0.0,  		0.0    , 0.0,
				0,   cos(alpha),-sin(alpha), 0.0,
				0.0, sin(alpha), cos(alpha), 0.0,
				0.0,   0.0, 	 	0.0, 	 1.0;
	return rotation;
}

Eigen::Matrix4d rotationY(const double &alpha){
	Eigen::Matrix4d rotation;
	rotation << cos(alpha),      0,    sin(alpha), 0.0,
					0, 			 1,       0.0,     0.0,
			   -sin(alpha),      0,    cos(alpha), 0.0,
					0.0		,   0.0, 	  0.0,     1.0;
	return rotation;
}

Eigen::Matrix4d rotationZ(const double &alpha){
	Eigen::Matrix4d rotation;
	rotation << cos(alpha), -sin(alpha),  0.0, 0.0,
				sin(alpha),  cos(alpha),  0.0, 0.0,
					0.0		,   0.0, 	  1.0, 0.0,
					0.0		,   0.0, 	  0.0, 1.0;
	return rotation;
}


//Class: Define properties and methods that the 5-DOF arm
//for Team D in mechatronics uses
class armController{
private:
	Eigen::Matrix4d H1o2i, H2o3i, H3o4i, H4o5i, H5o6i, g1i6;
	std::vector<std::unique_ptr<hebi::Module>> vec_module;
  	std::vector<std::string> moduleAddress;
  	long timeout_ms = 20;

  	double x1,x2,x3,x4,x5,x6;
  	double y1,y2,y3,y4,y5,y6;
  	double z1,z2,z3,z4,z5,z6;


public:
	armController();
	void initialize();
	void forwardKinematics(const std::vector<double> &alpha);
	bool inverseKinematics(const geometry_msgs::Pose &pose, std::vector<double> &alpha);
	bool sendCommand(const std::vector<double> &command);
	bool getFeedback(std::vector<double> &fbk,const int type);
};

armController::armController(){

	//Initialize transformation matrices using the home position of the (arms facing up)
	Eigen::Matrix4d T1o2i, T2o3i, T3o4i, T4o5i, T5o6i;

	//Cartesian offsets of each transformation in meters
	x1 = 0 ;    y1 =-0.102	  ; z1 = 0.067;
	x2 = 0;     y2 =0.321 	  ; z2 = 0;
	x3 = 0.324; y3 = 0    	  ; z3 = 0;
	x4 = 0.078; y4 = 0	  	  ; z4 = 0;
	x5 = 0	  ; y5 = 0	 	  ; z5 = 0;

    T1o2i << 1, 0, 0,  x1,
      		 0, 1, 0,  y1,
      		 0, 0, 1,  z1,
      		 0, 0, 0,  1;

    H1o2i = T1o2i*rotationX(PI/2);

    T2o3i << 1, 0, 0,  x2,
      		 0, 1, 0,  y2,
      		 0, 0, 1,  z2,
      		 0, 0, 0,  1;

    H2o3i = T2o3i*rotationZ(PI/2)*rotationX(PI);

    T3o4i << 1, 0, 0,  x3,
      		 0, 1, 0,  y3,
      		 0, 0, 1,  z3,
      		 0, 0, 0,  1;

    H3o4i = T3o4i*rotationX(PI);

    T4o5i << 1, 0, 0,  x4,
      		 0, 1, 0,  y4,
      		 0, 0, 1,  z4,
      		 0, 0, 0,  1;

    H4o5i = T4o5i*rotationY(PI/2)*rotationZ(PI/2);

    //set to identity until we have an end-effector  
    T5o6i <<  1, 0, 0,  x5,
      		 0, 1, 0,  y5,
      		 0, 0, 1,  z5,
     		 0, 0, 0,  1;

    H5o6i = T5o6i;
}
void armController::initialize(){
	vec_module.clear();
    vec_module.resize(4);

	// Setup the lookup
	long timeout = 4000; // Give the modules plenty of time to appear.
	hebi::Lookup lookup;
	hebi::MacAddress mac;

	printf("Looking up module by MAC address.\n");
	mac.setToHexString(std::string("D8:80:39:E8:DD:9E"));
	vec_module[0] = lookup.getModuleFromMac(mac, timeout);
	std::cout << (vec_module[0] ? "Found module." : "Module not found on network.") << std::endl;
	hebi_sleep_ms(100);

	printf("Looking up module by MAC address.\n");
	mac.setToHexString(std::string("D8:80:39:9A:84:F2"));
	vec_module[1] = lookup.getModuleFromMac(mac, timeout);
	std::cout << (vec_module[1] ? "Found module." : "Module not found on network.") << std::endl;
	hebi_sleep_ms(100);

	printf("Looking up module by MAC address.\n");
    mac.setToHexString(std::string("D8:80:39:9B:22:4F"));
	vec_module[2] = lookup.getModuleFromMac(mac, timeout);
	std::cout << (vec_module[2] ? "Found module." : "Module not found on network.") << std::endl;
	hebi_sleep_ms(100);

	printf("Looking up module by MAC address.\n");
	mac.setToHexString(std::string("D8:80:39:9B:2B:DC"));
	vec_module[3] = lookup.getModuleFromMac(mac, timeout);
	std::cout << (vec_module[3] ? "Found module." : "Module not found on network.") << std::endl;
	hebi_sleep_ms(100);

	//printf("Looking up module by MAC address.\n");
	//mac.setToHexString(std::string(""));
	//vec_module[4] = lookup.getModuleFromMac(mac, timeout);
}

//Function:: Calculate the forward kinematics of the arm in the lab
void armController::forwardKinematics(const std::vector<double> &alpha){
	g1i6 = rotationZ(alpha[0]) * H1o2i*
						   rotationZ(alpha[1])* H2o3i*
						   rotationZ(alpha[2])* H3o4i*
						   rotationZ(alpha[3])* H4o5i;

 
    std::cout << "Endeffector w.r.t arm base is : \n" << g1i6 << std::endl;
}

//Function:: Calculate the Inverse Kinematics of the arm in the lab
bool armController::inverseKinematics(const geometry_msgs::Pose &pose, std::vector<double> &alpha){
	Eigen::Quaternion<double> q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
	Eigen::Matrix4d g1i6_t;
    g1i6_t.setIdentity();
    //g1i6.block(0, 0, 3, 3) << q.matrix();
    g1i6_t.block(0, 0, 3, 3) << g1i6.block(0, 0, 3, 3);
    //g1i6.block(0, 3, 3, 1) << pose.position.x, pose.position.y, pose.position.z;
    g1i6_t.block(0, 3, 3,1) << g1i6.block(0, 3, 3, 1); 
    std::cout << "desired end-effector pose:" << std::endl << g1i6_t << std::endl;	

    //Solve for base angle. Set condition that if angle is greater  that PI
    //arm should got to negative values.

    //offset to apply to radius to get actual angle 0
    double offset_angle = asin(y1 / sqrt(g1i6_t(0, 3) * g1i6_t(0, 3) + g1i6_t(1, 3) *g1i6_t(1, 3)));
    std::cout << "offset angle is : " << offset_angle << std::endl;
    alpha[0] = atan2(g1i6_t(1, 3), g1i6_t(0, 3)) + offset_angle;
	    alpha[0] += PI;
	    if (alpha[0] >PI){
	    	alpha[0] = alpha[0] - 2*PI;
	    }

    //Solve for elevation and reach of 4th joint w.r.t base joint
    Eigen::Matrix4d g4o6 = H4o5i * rotationZ(alpha[4]) * H5o6i;
    Eigen::Matrix4d g1i4o = g1i6_t * g4o6.inverse();
    Eigen::Matrix4d g1i2i = rotationZ(alpha[0]) * H1o2i;
    Eigen::Matrix4d g2i4o = g1i2i.inverse() * g1i4o;
    double elevation = g2i4o(1, 3), reach = fabs(g2i4o(0, 3));


    std::cout << " wrist elevation is : " << elevation << std::endl;
    std::cout << " wrist reach is : " << reach << std::endl;

    //use law of cosines to find angle 2
    double L1 = fabs(y2); double L2 = fabs(x3);
    double L3 = sqrt(pow(reach,2)+pow(elevation,2));
    double cos_c = (pow(L3,2) - pow(L2,2) - pow(L1,2))/(2*L1*L2);
    
    //Elbow down configuration
    alpha[2] = -acos(cos_c);

    //solve for joint 2 angles
    double beta = asin(L2 / L3 * sin(alpha[2]));
    alpha[1] = atan2(reach,elevation) + beta;
    
    std::cout << asin(g1i6_t(2,2)) <<std::endl;
    if (g1i6_t(0,2) > 0){
    	alpha[3] =PI - alpha[1] + alpha[2]- (-PI/2 -asin(g1i6_t(2,2)));
    }
    else{
    	alpha[3] =PI - alpha[1] + alpha[2] + (-PI/2 -asin(g1i6_t(2,2)));
    } 



    //alpha[3] = M_PI - alpha[1] + alpha[2];

    std::cout << " angle 1 is : " << alpha[0] << std::endl;
    std::cout << " angle 2 is : " << alpha[1] << std::endl;
	std::cout << " angle 3 is : " << alpha[2] << std::endl;  
	std::cout << " angle 4 is : " << alpha[3] << std::endl;
	//std::cout << " angle 4 is : " << alpha[3] << std::endl; 




    return false;
}


bool armController::getFeedback(std::vector<double> &fbk,const int type){
  	hebi::Feedback feedback;
  	bool bSuccess = true;  
    	for (uint j = 0; j < 4; j++){
		  if (vec_module[j]->requestFeedback(&feedback,timeout_ms)){
  			switch (type){
  				case 1:
  					fbk[j] = feedback.actuator().position().get();
  					printf("Position feedback module %d = %lf rad\n",j+1,fbk[j]);
  					break;
  				case 2:
  					fbk[j] = feedback.actuator().velocity().get();
  					printf("Velocity feedback module %d = %lf rad/s\n",j+1,fbk[j]);
  					break;
  				case 3:
  					fbk[j] = feedback.actuator().torque().get();
  					printf("Torque feedback module %d = %lf N/m\n",j+1,fbk[j]);
  					break;
  			}
  		  }
          else{
            bSuccess = false;
        }
        if (bSuccess == false){
        printf("\nFeedback not recieved successfully module.\n");
        hebi_sleep_ms(timeout_ms);
    	}
  	
    }
  	return bSuccess;
}




int main(int argc, char* argv[]){

	std::vector<double> fbk_position(4);
	std::vector<double> alpha(4);
	geometry_msgs::Pose pose;

	pose.orientation.w = 0;
	pose.orientation.x = 1;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.position.x =  -.7138;
	pose.position.y = -0.0685;
	pose.position.z = -0.0119;


	bool success;
	ros::init(argc, argv, "arm_control");
	ros::NodeHandle nh;
	ros::Rate loop_rate(0.5);

	armController ac;
	ac.initialize();

	while (ros::ok()){

		ac.getFeedback(fbk_position,1);
		ac.forwardKinematics(fbk_position);

		success = ac.inverseKinematics(pose,alpha);






		loop_rate.sleep();
	}
  	return 0;
}