
#include "helper.h"
#include <stdint.h>
#include "Arduino.h"
#include <SPI.h>
#include "Escon.h"

///////////////////////Helper Functions///////////////////////  


// methods relating to arduino management
void motorClass::inputPins(int current, int speed, int enable, int direction, int encoder){
    currentCommandPin = current;
    speedFeedbackPin  = speed;
    enablePin         = enable;
    directionPin      = direction;
    encoderSlavePin   = encoder;
}

void motorClass::arduinoPinSetupMotor(void){
    pinMode(enablePin, OUTPUT);
    pinMode(directionPin, OUTPUT);
    pinMode(currentCommandPin, OUTPUT);
    digitalWrite(enablePin,HIGH);
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
    interrupts();             // enable all interrupts

    return AS5147P;
}

int motorClass::arduinoReadValues(void){
int analog_speed_value_bits;
analog_speed_value_bits = analogRead(speedFeedbackPin);
MotorVel = analog_speed_value_bits;
return analog_speed_value_bits;
}

void motorClass::arduinoWrite(void){
   int analogOutBits;
   analogOutBits = ANALOG_OUT_BITS*(ESCON_PWM_RANGE*currentCommand/ESCON_CURRENT_MAX + ESCON_PWM_MIN);
   analogWrite(currentCommandPin,analogOutBits);
   digitalWrite(directionPin,currentCommand>0);
}

void motorClass::inputVelocityPidGains(float proportional,float derivative,float integral){
    Kpv = proportional;
    Kdv = derivative;
    Kiv = integral;
}

int motorClass::openLoopController(void){
  //set scale factor value for to map from minimum pwm output to maximum pwm output
  currentCommand = desiredMotorVel * 0;//some_scale_factor;
  return 0;
}

void motorClass::storeOldVals(void){
  prevTime = currentTime;
  errorVelPrev = errorVel;
  encodercountPrev = encodercount;
}

void motorClass::calc_t(){
  currentTime = millis();
  dt = currentTime - prevTime;
}

float motorClass::motor_velocity_calc(void){
  calc_t();
  MotorVel = (encodercount-encodercountPrev) * dt;
  storeOldVals();
  return MotorVel; 
}

float motorClass::proportional_control(void){
  errorVel = desiredMotorVel - MotorVel;
  pCommand = Kpv * errorVel;
  return pCommand;
}

float motorClass::derivative_control(void){
  calc_t();
  dCommand = Kdv * (errorVel - errorVelPrev) / dt;
  return dCommand;
}

float motorClass::integral_control(void){
  calc_t();
  integratedVelError = integratedVelError + errorVel*dt;
  
  iCommand = Kiv*integratedVelError;

  //deal with integral windup
  if ( iCommand > MAX_PWM ){
    iCommand = MAX_PWM;
    integratedVelError = 0; //subject to change
  }
  else if ( iCommand < MIN_PWM ){
    iCommand = MIN_PWM;
    integratedVelError = 0;
  }
  return iCommand;
}

float motorClass::closedLoopController(void){
  currentCommand = proportional_control() + derivative_control();// + integral_control();
  return currentCommand; 
}

