
#include "helper.h"
#include <stdint.h>
#include "Arduino.h"
#include <SPI.h>
#include "Escon.h"

///////////////////////Helper Functions///////////////////////  


// methods relating to arduino management
void motorClass::inputPins(int command, int speedd, int current, int enable, int directionn, int encoder){
    CommandPin        = command;
    speedFeedbackPin  = speedd;
    currentFeedbackPin= current;
    enablePin         = enable;
    directionPin      = directionn;
    encoderSlavePin   = encoder;
}

void motorClass::arduinoPinSetupMotor(void){
    pinMode(enablePin, OUTPUT);
    pinMode(directionPin, OUTPUT);
    pinMode(CommandPin, OUTPUT);
    digitalWrite(enablePin,HIGH);
}
void motorClass::enableMotor(void){
    digitalWrite(enablePin,HIGH);
}
void motorClass::disableMotor(void){
    digitalWrite(enablePin,LOW);
}
void motorClass::initEncoder(void) {
  // Set slave selects as outputs
  pinMode(encoderSlavePin, OUTPUT);
  // Raise select pins
  // Communication begins when you drop the individual select signals
  digitalWrite(encoderSlavePin,HIGH);
}

unsigned int long motorClass::readEncoder(void){
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
    unsigned int AS5147P_angle;
    noInterrupts();           // disable all interrupts
    
    digitalWrite(encoderSlavePin,LOW);      // Begin SPI conversation
    delayMicroseconds(1);
    SPI.transfer16(0xFFFF);                     // Request count
    digitalWrite(encoderSlavePin,HIGH);     // Terminate SPI conversation     
    
    digitalWrite(encoderSlavePin,LOW);      // Begin SPI conversation
    delayMicroseconds(1);
    AS5147P_angle = SPI.transfer16(0xC000);           // Read highest order byte
    digitalWrite(encoderSlavePin,HIGH);     // Terminate SPI conversation
    SPI.endTransaction();
    AS5147P_angle = (AS5147P_angle & (0x3FFF));
      
    unsigned int long AS5147P = ( (unsigned long) AS5147P_angle);
    encodercount = AS5147P;

    if    ( encodercountPrev>ENC_THRESH_HIGH && encodercount < ENC_THRESH_LOW){
        encoderticker++;
    }
    else if (encodercountPrev<ENC_THRESH_LOW && encodercount > ENC_THRESH_HIGH){
        encoderticker--;
    }    
    encoderpos = 2.0*3.14159*(float(encodercount)/16384.0 + float(encoderticker));
    encodercountPrev = encodercount;
   
    
    interrupts();             // enable all interrupts
}

float motorClass::positionencoder(void){
 float pos = 1.0;
    return pos;
}

float motorClass::arduinoReadValuesSpeed(void){
int analog_speed_value_bits;
analog_speed_value_bits = analogRead(speedFeedbackPin);
MotorVel = 2*78.5*analog_speed_value_bits/818.4-78.5 + 0.23;
//return analog_speed_value_bits;
return MotorVel;
}


float motorClass::arduinoReadValuesCurrent(void){
int analog_current_value_bits;
analog_current_value_bits = analogRead(currentFeedbackPin);
MotorCurrent = 2*ESCON_CURRENT_MAX*analog_current_value_bits/818.4-ESCON_CURRENT_MAX ;
//return analog_speed_value_bits;
return MotorCurrent;
}

int motorClass::arduinoWriteCurrent(void){
   int analogOutBits;
   float pwm_duty;
   pwm_duty      = (ESCON_PWM_RANGE/ESCON_CURRENT_MAX)*abs(currentCommand) + ESCON_PWM_MIN;
   analogOutBits = ANALOG_OUT_BITS*(pwm_duty);
   analogWrite(CommandPin,analogOutBits);
   digitalWrite(directionPin,currentCommand>0);
   return currentCommand>0;
}


int motorClass::arduinoWriteSpeed(void){
   int analogOutBits;
   float pwm_duty;
   pwm_duty      = (ESCON_PWM_RANGE/ESCON_SPEED_MAX)*abs(speedCommand) + ESCON_PWM_MIN;
   analogOutBits = ANALOG_OUT_BITS*(pwm_duty);
   analogWrite(CommandPin,analogOutBits);
   digitalWrite(directionPin,speedCommand>0);
   return speedCommand>0;
}

void motorClass::inputVelocityPidGains(float proportional,float integral,float tau_derivative,float alpha_derivative){
    Kpv = proportional;
    tau_i = integral;
    tau_d = tau_derivative;
    alpha_d = alpha_derivative;
}


void motorClass::storeOldVals(void){
  prevTime = currentTime;
  errorVelPrev = errorVel;
  encodercountPrev = encodercount;
}

void motorClass::calc_t(){
  currentTime = millis();
  //dt = 0.001*(currentTime - prevTime);
   dt = 0.002;
}

/*float motorClass::motor_velocity_calc(void){
  calc_t();
  MotorVel = (encodercount-encodercountPrev) * dt;
  storeOldVals();
  return MotorVel; 
}*/

/*
float motorClass::closedLoopControllerCurrent(void){
  float y_I;
  float e_d;
  float y_d;
  float term_test;
  calc_t();

  errorVel = desiredMotorVel - MotorVel;
  
  y_I = Kpv*0.5*Kiv*dt*(errorVel+errorVelPrev) + y_I_minus1;
  
  if ( y_I > MAX_i ){
    y_I = 0.02*MAX_i;
  }
  else if ( y_I < MIN_i ){
    y_I  = 0.02*MIN_i;
  }
  
  if ( dt < 0.001 ){
    dt = 1/250.0;
  }
  e_d = Kpv*errorVel + y_I;
  y_d = y_d_minus1*(2*tau_d/dt -1.0) + e_d_minus1*(1.0 - 2*alpha_d*tau_d/dt)+ e_d*(2*alpha_d*tau_d/dt +1.0); //y_d_minus1*(2*tau_d/dt -1.0)
  currentCommand = y_d;


  prevTime = currentTime;
  errorVelPrev = errorVel;
  encodercountPrev = encodercount;
  
  y_I_minus1 = y_I;
  e_d_minus1 = e_d;
  y_d_minus1 = y_d;
  
  return y_d; 
}
*/
float motorClass::closedLoopControllerSpeed(void){
  speedCommand = desiredMotorVel;

  float y_I;
  float e_d;
  float y_d;
  float term_test;
  calc_t();

  errorVel = desiredMotorVel - MotorVel;
  
  y_I = Kpv*0.5*dt*(errorVel+errorVelPrev)/tau_i + y_I_minus1;
  
  if ( y_I > MAX_i ){
    y_I = MAX_i;
  }
  else if ( y_I < MIN_i ){
    y_I  = MIN_i;
  }
  
  e_d = Kpv*errorVel + y_I;
  y_d = (1.0/((2.0*alpha_d*tau_d/dt) + 1.0))*(y_d_minus1*((2.0*alpha_d*tau_d/dt) - 1.0) + e_d*((2.0*tau_d/dt) + 1.0) + e_d_minus1*(1.0 -2.0*tau_d/dt));
   
  //y_d = y_d_minus1*(2*(1/alpha_d)*tau_d/dt -1.0) + e_d_minus1*(1.0 - 2*tau_d/dt)+ e_d*(2*tau_d/dt +1.0); //y_d_minus1*(2*tau_d/dt -1.0)


  prevTime = currentTime;
  errorVelPrev = errorVel;
  encodercountPrev = encodercount;
  
  y_I_minus1 = y_I;
  e_d_minus1 = e_d;
  y_d_minus1 = y_d;
  
  currentCommand = e_d;
  //currentCommand = Kpv*errorVel;
  //currentCommand = dt;
  return currentCommand;
}

float motorClass::closedLoopControllerInternalRes(void){
    float R  =  ARM_BIAS/60.0;
    
   currentCommand = ARM_BIAS - MotorVel*R;
   
  if ( MotorVel > ARM_BIAS/R  ){
   currentCommand = 0;
  }
  else if (MotorVel < 0){
    currentCommand = ARM_BIAS;
  }    
    return currentCommand; 
}

void motorClass::setdesiredMotorVel(float desiredVel){
  desiredMotorVel = desiredVel;
}


void motorClass::CLC(void){
    if ( mode == 0 ){
   closedLoopControllerSpeed();
  }
  else if (mode == 1 ){
   closedLoopControllerInternalRes();
  }   
}




















