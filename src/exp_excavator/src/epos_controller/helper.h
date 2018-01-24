#ifndef EPOS_HELPER_H
#define EPOS_HELPER_H
//EPOS driver Structure
class EposManager {
public:
	EposManager();
	~EposManager();	
	
	void spdComCallback(const exp_excavator::JointCommandConstPtr& msg);
	
    /*void GetCurrentPositionAllDevice(double* CurrentPosition);
    void GetCurrentVelocityAllDevice(double* CurrentVelocity);
    void GetCurrentCurrentAllDevice(double* CurrentCurrent);*/

    void GetCurrentPosition(void *keyHandle_, int *CurrentPosition, unsigned short nodeId);
    void GetCurrentVelocity(void *keyHandle_, int *CurrentVelocity, unsigned short nodeId);
    void GetCurrentCurrent(void *keyHandle_, short *CurrentCurrent, unsigned short nodeId);
        
	double motorTHETA;
	double motorOMEGA;
	double motorCURRENT;
	
	bool Kmode;
	bool Lastmode;
	
private:
	void *pHandle;
	unsigned short nodeIdDevice;
};


//Constants
const int MAX_PWM = 400;
const int MIN_PWM = -400;
//Staff constants
const int   FREQ = 2000;
const float PERIOD = 0.0005;
const float PERIOD_MICROS = (PERIOD * 1000000.0);
const int   SERIAL_FREQ = 100;  //Should be at least 10 times greater than sin wave frequency
const float SERIAL_PERIOD = 0.01;
const float SERIAL_PERIOD_MICROS = 10000.0;

//Staff functions
void stopIfFault(void);
void DoSerialSend(void);

#endif //HELPER_H
