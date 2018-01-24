#include <ros.h>
#include "Escon.h"
#include "helper.h"
#include <std_msgs/UInt16.h>
#include <SPI.h>

unsigned int currentCommandArm;
unsigned int currentCommandBoom;

int arduinoPrescalerEraser = 7;
int arduinoClockPrescaler = 3;     // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.   

motorClass motorArm;

ros::NodeHandle  nh;

std_msgs::UInt16 enc_msg;
ros::Publisher chatter("chatter", &enc_msg);

void setup()
{
    motorArm.inputPins(PWM_CURRENT_COMMAND_PIN_ARM, ANALOG_SPEED_PIN_ARM, ENABLE_PIN_ARM, DIRECTION_PIN_ARM,SLAVE_SELECT_ENCODER_PIN_ARM);
    motorArm.arduinoPinSetupMotor();
    motorArm.inputVelocityPidGains(K_v_PROPORTIONAL,K_v_DERIVATIVE,K_v_INTEGRAL);//Set Gains
    motorArm.initEncoder();
    
  
    TCCR2B &= ~arduinoPrescalerEraser;
    TCCR2B |= arduinoClockPrescaler;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 011
  
    SPI.begin();
    motorArm.readEncoder();

    nh.initNode(); //start ROS node
    nh.advertise(chatter); //advertise topic

    motorArm.calc_t();
}

void loop()
{ //motorArm.encodercount = readEncoder(1);
  //motorBoom.encodercount = readEncoder(2);

  //Definitions of closedLoopControl() and storeOldVals in helper.cpp
      //enc_msg.data = readEncoder(1);

  motorArm.storeOldVals();


  motorArm.readEncoder();
  enc_msg.data = motorArm.arduinoReadValues();
  motorArm.closedLoopController();
  motorArm.arduinoWrite();
  
  chatter.publish( &enc_msg );
  
  nh.spinOnce();
}




