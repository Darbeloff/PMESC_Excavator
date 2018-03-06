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
    float Kiv = 0;
    
    unsigned long encodercountPrev = 0;
    int encodertickerPrev = 0;
    
    float errorVel = 0;
    float errorVelPrev = 0;
    float integratedVelError = 0;
    float y_I_minus1 = 0;
    float e_d_minus1 = 0;
    float y_d_minus1 = 0;
    float pCommand = 0;
    float dCommand = 0;
    float iCommand = 0;
    
    float speedCommand = 0;
    
    float desiredMotorVel = 0;
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
    float currentCommand = 0;
    
    void inputVelocityPidGains(float proportional,float integral,float tau_derivative,float alpha_derivative);
    void inputPins(int command, int speedd,int current, int enable, int directionn, int encoder);
    void arduinoPinSetupMotor(void);
    
    void storeOldVals(void);
    void calc_t(void);
    float motor_velocity_calc(void);
    
    float proportional_control(void);
    float derivative_control(void);
    float integral_control(void);
    
    float closedLoopControllerCurrent(void);
    float closedLoopControllerSpeed(void);
    float closedLoopControllerInternalRes(void);
    void setdesiredMotorVel(float desiredVel);
        
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
const float K_v_PROPORTIONAL = 0.011;
const float K_v_INTEGRAL     = 50;
const float ALPHA_d          = 10;
const float TAU_d            = 0.0007;

const float ARM_BIAS         = 2.5;

const int ENC_THRESH_HIGH    = 15000;
const int ENC_THRESH_LOW     = 1000;
//Constants
const int MAX_i = 1.0;
const int MIN_i = -1.0;
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
