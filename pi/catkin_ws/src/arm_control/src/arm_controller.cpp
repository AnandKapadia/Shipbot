#include <stdio.h>
#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
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
#include "arm_kinematics.h"
#include "arm_control/arm_traj.h"

#define PI 3.14159

//Class: Define properties and methods that the 5-DOF arm
//for Team D in mechatronics uses
class armController{
private:
	std::vector<std::unique_ptr<hebi::Module>> vec_module;
  	std::vector<std::string> moduleAddress;
  	int numModules;
  	long timeout_ms = 20;


public:
	armKinematics ak;
	armController();
	void initialize();
	void sendCommand(const std::vector<double> &command);
	void sendCommand(const std::vector<double> &pos,const std::vector<double> &vel);
	void sendCommand(const std::vector<double> &pos,const std::vector<double> &vel,const std::vector<double> &torque);
	bool getFeedback(std::vector<double> &fbk,const int type);
	bool execute(arm_control::arm_traj::Request &req, arm_control::arm_traj::Response &res); 
};

armController::armController(){

	numModules = 5;

}
void armController::initialize(){
	vec_module.clear();
    vec_module.resize(5);

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

	printf("Looking up module by MAC address.\n");
	mac.setToHexString(std::string("D8:80:39:9A:B8:11"));
	vec_module[4] = lookup.getModuleFromMac(mac, timeout);
	std::cout << (vec_module[4] ? "Found module." : "Module not found on network.") << std::endl;
	hebi_sleep_ms(100);
}

void armController::sendCommand(const std::vector<double> &command){
hebi::Command cmd;
    bool bSuccess = true;
    for(uint j = 0; j < 5; j++){
          cmd.actuator().position().set(command[j]); 
          printf("send command to joint %d with angle %f rad \n", j, command[j]);
          if(!vec_module[j]->sendCommandWithAcknowledgement(cmd, timeout_ms)){
             bSuccess = false;
            printf("Did not receive acknowledgement from %d!\n",j);
          }
          else{
          //printf("Got acknowledgement.\n");
          }
    }
 }

void armController::sendCommand(const std::vector<double> &pos,const std::vector<double> &vel){
	hebi::Command cmd;
    bool bSuccess = true;
    for(uint j = 0; j < 5; j++){
          cmd.actuator().position().set(pos[j]);
          cmd.actuator().velocity().set(vel[j]);  
          printf("send command to joint %d with angle %f rad and velocity %f rad/s \n", j, pos[j], vel[j]);
          if(!vec_module[j]->sendCommandWithAcknowledgement(cmd, timeout_ms)){
             bSuccess = false;
            printf("Did not receive acknowledgement from %d!\n",j);
          }
          else{
          //printf("Got acknowledgement.\n");
          }
    }
 }

 void armController::sendCommand(const std::vector<double> &pos,const std::vector<double> &vel,const std::vector<double> &torque){
 	hebi::Command cmd;
    bool bSuccess = true;
    for(uint j = 0; j < 5; j++){
          cmd.actuator().position().set(pos[j]);
          cmd.actuator().velocity().set(vel[j]); 
          cmd.actuator().velocity().set(torque[j]); 
          printf("send command to joint %d with angle %f rad and velocity %f rad/s \n", j, pos[j], vel[j]);
          if(!vec_module[j]->sendCommandWithAcknowledgement(cmd, timeout_ms)){
             bSuccess = false;
            printf("Did not receive acknowledgement from %d!\n",j);
          }
          else{
          //printf("Got acknowledgement.\n");
          }
    }
 }

bool armController::getFeedback(std::vector<double> &fbk,const int type){
  	hebi::Feedback feedback;
  	bool bSuccess = true;  
    	for (uint j = 0; j < 5; j++){
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
        }
        if (bSuccess == false){
        printf("\nFeedback not recieved successfully module.\n");
        hebi_sleep_ms(timeout_ms);
    	}
    	
  	return bSuccess;
  }

bool armController::execute(arm_control::arm_traj::Request &req, arm_control::arm_traj::Response &res){

	std::vector<double> joint_pos(5); 		//vector of joint angles
	std::vector<double> joint_vel(5);	//vector of joint velocities
	std::vector<double> fbk_position(5);
	std::vector<double> fbk_velocity(5);
	std::vector<double> joint_pos_end(5);

	getFeedback(fbk_position,1);

	//assign start and end positions from executive function
	joint_pos = fbk_position;
	ak.inverseKinematics(req.endPose,joint_pos_end);

	//get the minimum jerk coefficients for given trajectory
	Eigen::MatrixXd J;
	Eigen::MatrixXd J1;
	J = ak.MinimumJerkCoefficients(joint_pos,joint_pos_end);

	//start timer for minimum jerk trajectory
	double begin = ros::Time::now().toSec();
	double t = 0;

	bool reach_goal = false;
	while(ros::ok()){	
		double sum = 0.0;
		if(ak.inverseKinematics(req.endPose,joint_pos_end)){
			for (uint j = 0; j<5;j++){
				J1 = ak.MinimumJerkCoefficients(joint_pos,joint_pos_end);
				//Joint angles from minimum jerk trajectory
				joint_pos[j] = J(0,j)*pow(t,5) + J(1,j)*pow(t,4) + J(2,j)*pow(t,3) + J(3,j)*pow(t,2) +
					   	 J(4,j)*pow(t,1) + J(5,j);

				//Joint Velocity from minimum jerk trajectory
				joint_vel[j] = 5*J(0,j)*pow(t,4) + 4*J(1,j)*pow(t,3) + 3*J(2,j)*pow(t,2) + 2*J(3,j)*pow(t,1) +
					    	J(4,j)*pow(t,1);
			}


			//send joint commands
			std::vector<double> joint_torque(5);
			joint_torque = {0,.3,.1,.03,0};
			sendCommand(joint_pos,joint_vel,joint_torque);
		}
		else{
		std::cout << " given trajectory falls outside workspace" << std::endl;
		res.result = 1;
		}

		getFeedback(fbk_position,1);
		getFeedback(fbk_velocity,2);
		for (uint j; j<5;j++){
			sum +=fbk_velocity[j];
		}

		std::cout << "sum is : " <<sum << std::endl;

		if(fabs(fbk_position[0] - joint_pos_end[0]) < .05 && 
			fabs(fbk_position[1] - joint_pos_end[1]) < .05 && 
	   		fabs(fbk_position[2] - joint_pos_end[2]) < .05 
	   		 || t>1.2 && fabs(J1(0,1)) < .00001)

	   		 {
	   		getFeedback(fbk_position,1);
	   		ak.inverseKinematics(req.endPose,joint_pos_end);
			res.result = 0;
			return 1;
		}
		double end = ros::Time::now().toSec();
		t = (end-begin)/2;
		std::cout << " time is : " << t << std::endl;
	

	}
	return 0;
}

int main(int argc, char* argv[]){


	//initialize server
	ros::init(argc, argv, "arm_trajectory_server");
    ros::NodeHandle nh;


    armController ac;
    ac.initialize();

    //advertise service to topic
   	ros::ServiceServer service = nh.advertiseService("arm_control/arm_traj", &armController::execute, &ac); 
	
	ros::spin();

  	return 0;
}