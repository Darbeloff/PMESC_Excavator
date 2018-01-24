/*
 * epos_manager2.cpp
 *
 *  Created on: Feb 22, 2017
 *      Author: fes
 */

#include <iostream>
#include <stdio.h>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <epos_library/Definitions.h>
#include <exp_excavator/JointValues.h>
#include <exp_excavator/JointCommand.h>

using namespace std;

class EposManager {
public:
	EposManager();
	~EposManager();	
	
   /*void spdComCallback(const exp_excavator::JointCommandConstPtr& msg);
	void GetCurrentPositionAllDevice(double* CurrentPosition);
    void GetCurrentPosition(void *keyHandle_, int *CurrentPosition, unsigned short nodeId);
    void GetCurrentVelocityAllDevice(double* CurrentVelocity);
    void GetCurrentVelocity(void *keyHandle_, int *CurrentVelocity, unsigned short nodeId);
	void GetCurrentCurrentAllDevice(double* CurrentCurrent);
    void GetCurrentCurrent(void *keyHandle_, short *CurrentCurrent, unsigned short nodeId); */
        
	ros::NodeHandle n;
	/*ros::Subscriber subSpdCom;
	ros::Publisher pubPosIs;*/
	ros::Timer timer;

	/* double motorTHETA[2];
	double motorOMEGA[2];
	double motorCURRENT[2];*/
	
    /*bool Kmode;
	bool Lastmode;*/

    float EPOS_ARM_SCALING
	short EPOS_ARM_OFFSET

private:
	void *pHandleArm;
	unsigned short nodeIdArm;
	unsigned short analogInputNbArm;

};

EposManager::EposManager() {
	unsigned int err;
	char *pDeviceName = (char*)"EPOS2";
	char *pProtocolStackName = (char*)"MAXON SERIAL V2";
	char *pInterfaceName = (char*)"USB";
	char *pPortNameArm = (char*)"USB1";
        
	nodeIdArm = 2;
	analogInputNbArm = 1;

    EPOS_ARM_SCALING = 2000;
    EPOS_ARM_OFFSET = -5000;
	
	pHandleArm = VCS_OpenDevice(pDeviceName, pProtocolStackName,pInterfaceName, pPortNameArm, &err);
	VCS_ClearFault(pHandleArm,nodeIdArm, &err);

    VCS_AnalogInputConfiguration(pHandleArm,nodeIdArm,analogInputNbArm,AIC_ANALOG_CURRENT_SETPOINT,1,1,&err);
	
	VCS_SetDisableState(pHandleArm, nodeIdArm, &err);
	
	VCS_SetOperationMode(pHandleArm, nodeIdArm, OMD_CURRENT_MODE, &err);

    VCS_ActivateAnalogCurrentSetpoint(pHandleArm, nodeIdArm, analogInputNbArm , EPOS_ARM_SCALING, EPOS_ARM_OFFSET, &err);

    VCS_EnableAnalogCurrentSetpoint(pHandleArm, nodeIdArm, &err);
	
	VCS_SetEnableState(pHandleArm, nodeIdArm, &err);
	
}

EposManager::~EposManager() {
	unsigned int err;

	VCS_SetDisableState(pHandleArm, nodeIdArm, &err);
}


void EposManager::GetCurrentPosition(void *keyHandle_, int *CurrentPosition, unsigned short nodeId){
        unsigned int err;
        VCS_GetPositionIs(keyHandle_, nodeId, CurrentPosition, &err); 
    
        if(err) {
	        ROS_ERROR("EPOS for Arm GetPositionIS Error:0x%08x", err);
        }
}

void EposManager::GetCurrentVelocity(void *keyHandle_, int *CurrentVelocity, unsigned short nodeId){
        unsigned int err;
        VCS_GetVelocityIsAveraged(keyHandle_, nodeId, CurrentVelocity, &err); 
    
        if(err) {
	        ROS_ERROR("EPOS for Arm GetVelocityIS Error:0x%08x", err);
        }
}

void EposManager::GetCurrentCurrent(void *keyHandle_, short *CurrentCurrent, unsigned short nodeId){
        unsigned int err;
        VCS_GetCurrentIsAveraged(keyHandle_, nodeId, CurrentCurrent, &err); 
    
        if(err) {
	        ROS_ERROR("EPOS for Arm GetVelocityIS Error:0x%08x", err);
        }
}

void EposManager::GetCurrentPositionAllDevice(double* CurrentPosition){
	int Pos;
	GetCurrentPosition(pHandleBoom, &Pos,nodeIdBoom);
	CurrentPosition[0]=(-2*3.1415926/(50*72))*Pos;
	GetCurrentPosition(pHandleArm, &Pos,nodeIdArm);
	CurrentPosition[1]=(-2*3.1415926/(50*72))*Pos;
}

void EposManager::GetCurrentVelocityAllDevice(double* CurrentVelocity){
	int Vel;
	GetCurrentVelocity(pHandleBoom, &Vel,nodeIdBoom);
	CurrentVelocity[0]=(-2*3.1415926/(50*72))*Vel;
	GetCurrentVelocity(pHandleArm, &Vel,nodeIdArm);
	CurrentVelocity[1]=(-2*3.1415926/(50*72))*Vel;
}

void EposManager::GetCurrentCurrentAllDevice(double* CurrentCurrent){
	short Cur;
	GetCurrentCurrent(pHandleBoom, &Cur,nodeIdBoom);
	CurrentCurrent[0]=Cur;
	GetCurrentCurrent(pHandleArm, &Cur,nodeIdArm);
	CurrentCurrent[1]=Cur;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "epos_manager");
        
	
	EposManager em;
	ROS_INFO("EPOS Manager has started.");
	ros::Rate r(100);
	/*em.pubPosIs = em.n.advertise<sensor_msgs::JointState>("/joint_states_EPOS",1);*/


	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	while(ros::ok()){
	
	    /*sensor_msgs::JointState msg2;
        msg2.header.stamp = ros::Time::now();
        em.GetCurrentPositionAllDevice(em.motorTHETA);
        em.GetCurrentVelocityAllDevice(em.motorOMEGA);
        em.GetCurrentCurrentAllDevice(em.motorCURRENT);
        msg2.name.push_back("Boom");
        msg2.position.push_back(em.motorTHETA[0]);
        msg2.velocity.push_back(em.motorOMEGA[0]);
        msg2.effort.push_back(em.motorCURRENT[0]);
        msg2.name.push_back("Arm");
        msg2.position.push_back(em.motorTHETA[1]);
        msg2.velocity.push_back(em.motorOMEGA[1]);
        msg2.effort.push_back(em.motorCURRENT[1]);
        em.pubPosIs.publish(msg2); */
        
        }
        return 0;
}
