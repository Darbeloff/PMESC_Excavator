#ifndef HELPER_H
#define HELPER_H
#include <std_msgs/Float32.h>

//Motor Structure
class motorClass
{
  private:
    unsigned long currentTime = 0;
    unsigned long prevTime = 0;
    float dt = 0; 
    
    float Kpv = 0;
    float tau_d = 0;
    float alpha_d = 0;
    float tau_i = 0;

    float Kp_cascade = 0;

    unsigned long encodercountPrev = 0;
    int encodertickerPrev = 0;
    

    float integratedVelError = 0;
    
    float e_d_minus1 = 0;
    float y_d_minus1 = 0;



    float avgMotorVel;
    int CommandPin;
    
    int speedFeedbackPin;
    int currentFeedbackPin;
    int enablePin;
    int directionPin;
    int encoderSlavePin;
  public:    
  
    unsigned long encodercount = 0;
    int encoderticker = 0;
    
    float encoderpos = 0;
    float MotorVel;
    float MotorCurrent;

    float errorPos = 0;
    float errorPosPrev = 0;  
    
    float errorVel = 0;
    float errorVelPrev = 0;    
    
    float e_filt = 0;
    float e_filt_minus1 = 0;
    float y_I_minus1 = 0;
    
    float speedCommand = 0;
    float currentCommand = 0;
    
    float positionCalibration = 0;
    float referenceMotorVel = 0;
    float referencePosition = 0;
    
    int mode = 3; // modes: 0-Manual speed reference mode, 1-Internal res mode , 2-Position Control Mode
    int last_mode = 3;
    void inputPositionCascadeGain(float proportional);
    void inputVelocityPidGains(float proportional,float integral,float tau_derivative,float alpha_derivative);
    void inputPins(int command, int speedd,int current, int enable, int directionn, int encoder);
    void arduinoPinSetupMotor(void);
    
    void storeOldVals(void);
    void calc_t(void);
    float motor_velocity_calc(void);
    
    float proportional_control(void);
    float derivative_control(void);
    float integral_control(void);

    void  CLC(void);
    float closedLoopControllerCurrent(void);
    float closedLoopControllerSpeed(void);
    float closedLoopControllerInternalRes(void);
    float closedLoopControllerPosition(void);
    float closedLoopControllerPositionCascade(void);
    float closedLoopControllerSpeedReference(void);

    
    //void setReferenceMotorVel(float referenceVel);
        
    void enableMotor(void);
    void disableMotor(void);
    
    void sendCurrentCommandArduino(void);

    void initEncoder(void); 
    unsigned int long readEncoder(void);
    float positionencoder(void);
    
    float arduinoReadValuesSpeed(void);
    float arduinoReadValuesCurrent(void);

    int arduinoWriteCurrent(void);
    int arduinoWriteSpeed(void);

};
//
// V PID gains
const float K_PROPORTIONAL_boom = 1.8;
const float K_PROPORTIONAL_arm  = 0.25;

const float TAU_i_arm            = 0.6;
const float TAU_i_boom           = 0.2;

const float ALPHA_d_arm          = 0.1;
const float ALPHA_d_boom         = 0.1;

const float TAU_d_arm            = 0.01;
const float TAU_d_boom           = 0.01;

//Cascade Gains
const float K_PROPORTIONAL_CASCADE_arm = 5;
const float K_PROPORTIONAL_CASCADE_boom = 5;

//Internal Resistance Gain
const float ARM_BIAS         = 5.0;

const int ENC_THRESH_HIGH    = 13000;
const int ENC_THRESH_LOW     = 2000;
//Constants
const int MAX_i = 2.0;
const int MIN_i = -2.0;
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
