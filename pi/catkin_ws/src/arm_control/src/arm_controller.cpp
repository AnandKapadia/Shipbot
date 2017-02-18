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
	Eigen::Matrix4d H1o2i, H2o3i, H3o4i, H4o5i, H5o6i;
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
	bool inverseKinematics(geometry_msgs::Pose pose, std::vector<double> &alpha);
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


void armController::forwardKinematics(const std::vector<double> &alpha){
	Eigen::Matrix4d g1i6 = rotationZ(alpha[0]) * H1o2i*
						   rotationZ(alpha[1])* H2o3i*
						   rotationZ(alpha[2])* H3o4i*
						   rotationZ(alpha[3])* H4o5i;

 
    std::cout << "Endeffector position is : \n" << g1i6 << std::endl;
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
  					//printf("Velocity feedback module %d = %lf rad/s\n",j+1,fbk[j]);
  					break;
  				case 3:
  					fbk[j] = feedback.actuator().torque().get();
  					//printf("Torque feedback module %d = %lf N/m\n",j+1,fbk[j]);
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

	ros::init(argc, argv, "arm_control");
	ros::NodeHandle nh;
	ros::Rate loop_rate(0.5);

	armController ac;
	ac.initialize();

	while (ros::ok()){

		ac.getFeedback(fbk_position,1);
		ac.forwardKinematics(fbk_position);



	loop_rate.sleep();
	}
  	return 0;
}