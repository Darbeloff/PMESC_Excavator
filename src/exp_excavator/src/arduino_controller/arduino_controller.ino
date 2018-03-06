#include <ros.h>
#include "Escon.h"
#include "helper.h"
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>

#include <SPI.h>

unsigned int currentCommandArm;
unsigned int currentCommandBoom;
float v_desired;

int arduinoPrescalerEraser = 7;
int arduinoClockPrescaler = 3;     // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.   

unsigned long i = 0;
unsigned long time_last = millis();
unsigned long time_now  = millis();

motorClass motorArm;

ros::NodeHandle  nh;

std_msgs::Float32 hz_loop_msg;
std_msgs::Float32 vel_actual_msg;
std_msgs::Float32 cur_actual_msg;
std_msgs::UInt16 enc_actual_msg;

ros::Publisher loop_freq("loop_freq",  &hz_loop_msg);
ros::Publisher vel_actual("vel_actual",  &vel_actual_msg);
ros::Publisher cur_actual("cur_actual",  &cur_actual_msg);
ros::Publisher enc_actual("enc_actual", &enc_actual_msg);

void referenceCb(const std_msgs::Empty& msg)
{ vel_actual_msg.data = motorArm.MotorVel;
  cur_actual_msg.data = motorArm.currentCommand;
  enc_actual_msg.data = motorArm.encoderpos;

  vel_actual.publish( &vel_actual_msg );
  cur_actual.publish( &cur_actual_msg );
  enc_actual.publish( &enc_actual_msg ); 

  hz_loop_msg.data = float(i)/(0.001*float(millis()-time_last));
  i=0;
  time_last = millis();
  loop_freq.publish( &hz_loop_msg);
}

ros::Subscriber<std_msgs::Empty> reference_sub("reference", &referenceCb);

void setup()
{
    motorArm.inputPins(PWM_COMMAND_PIN_ARM, ANALOG_SPEED_PIN_ARM, ANALOG_CURRENT_PIN_ARM, ENABLE_PIN_ARM, DIRECTION_PIN_ARM,SLAVE_SELECT_ENCODER_PIN_ARM);
    motorArm.arduinoPinSetupMotor();
    motorArm.inputVelocityPidGains(K_v_PROPORTIONAL,K_v_INTEGRAL,TAU_d,ALPHA_d);//Set Gains
    motorArm.initEncoder();
    
   
  
    TCCR2B &= ~arduinoPrescalerEraser;
    TCCR2B |= arduinoClockPrescaler;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 011
  
    SPI.begin();
    motorArm.readEncoder();

    nh.initNode(); //start ROS node
    nh.advertise(loop_freq);
    nh.advertise(vel_actual); //advertise topic
    nh.advertise(cur_actual); 
    nh.advertise(enc_actual);
    nh.subscribe(reference_sub);

    motorArm.enableMotor();

    motorArm.calc_t();
    
}

void loop()
{ 
  motorArm.storeOldVals();

  motorArm.readEncoder();
  motorArm.arduinoReadValuesSpeed();
  motorArm.arduinoReadValuesCurrent();
 
 
  v_desired = 50*sign(sin(0.0005*millis()));
  motorArm.setdesiredMotorVel(v_desired);
  
  //motorArm.closedLoopControllerInternalRes();
  motorArm.closedLoopControllerSpeed();
  motorArm.arduinoWriteCurrent();

//  vel_actual.publish( &vel_actual_msg );
//  vel_desired.publish( &vel_desired_msg );
  i++;
  nh.spinOnce();
  delay(1);
}

float sign(float value){
  return float((value>0)-(value<0));
}

