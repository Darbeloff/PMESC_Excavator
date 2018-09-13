#include <ros.h>
#include "Escon.h"
#include "helper.h"
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <exp_excavator/JointStateMachineArduino.h>
#include <exp_excavator/JointCommandArduino.h>

#include <SPI.h>


unsigned int currentCommandArm;
unsigned int currentCommandBoom;
float v_desired;
float dither = 0;
float dither_last = 0;
float pos_initial_arm = 0;
float arm_power_gradient = 0;

int arduinoPrescalerEraser = 7;
int arduinoClockPrescaler = 3;     // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.

unsigned long i = 0;
unsigned long time_last = millis();
unsigned long time_now  = millis();

motorClass motorArm;
motorClass motorBoom;

ros::NodeHandle  nh;

std_msgs::Float32 hz_loop_msg;
exp_excavator::JointStateMachineArduino joint_states_msg;

ros::Publisher loop_freq("loop_freq"  ,  &hz_loop_msg);
ros::Publisher machine_state_arduino("machine_state_arduino",  &joint_states_msg);

void loggingCb(const std_msgs::Empty& msg)
{ joint_states_msg.armV  = motorArm.MotorVel;
  joint_states_msg.boomV = motorBoom.MotorVel;

  joint_states_msg.armP  = motorArm.encoderpos;
  joint_states_msg.boomP = motorBoom.encoderpos;

  joint_states_msg.armI  = motorArm.MotorCurrent;
  joint_states_msg.boomI = motorBoom.MotorCurrent;

  joint_states_msg.armMode  = motorArm.mode;
  joint_states_msg.boomMode = motorBoom.mode;

  hz_loop_msg.data = float(i) / (0.001 * float(millis() - time_last));
  i = 0;
  time_last = millis();

  machine_state_arduino.publish( &joint_states_msg);
  loop_freq.publish( &hz_loop_msg);
}

void calibrationCb(const std_msgs::Float32MultiArray& msg)
{
  motorBoom.positionCalibration = msg.data[0];
  motorArm.positionCalibration  = msg.data[1];
}

void commandCb(const exp_excavator::JointCommandArduino& msg)
{
  motorBoom.referenceMotorVel =  msg.boomV;
  motorArm.referenceMotorVel  =  msg.armV;
  
  motorBoom.mode = msg.BoomMode;
  motorArm.mode  = msg.ArmMode;

}

ros::Subscriber<std_msgs::Empty> logging_sub("logging_ping", &loggingCb);
ros::Subscriber<exp_excavator::JointCommandArduino> arduino_command("arduino_commands", &commandCb);

void setup()
{
  motorArm.inputPins(PWM_COMMAND_PIN_ARM,   ANALOG_SPEED_PIN_ARM,  ANALOG_CURRENT_PIN_ARM,  ENABLE_PIN_ARM,  DIRECTION_PIN_ARM,  SLAVE_SELECT_ENCODER_PIN_ARM);
  motorBoom.inputPins(PWM_COMMAND_PIN_BOOM, ANALOG_SPEED_PIN_BOOM, ANALOG_CURRENT_PIN_BOOM, ENABLE_PIN_BOOM, DIRECTION_PIN_BOOM, SLAVE_SELECT_ENCODER_PIN_BOOM);

  motorArm.arduinoPinSetupMotor();
  motorBoom.arduinoPinSetupMotor();

  motorArm.inputVelocityPidGains(K_PROPORTIONAL_arm, TAU_i_arm, TAU_d_arm, ALPHA_d_arm); //Set Gains
  motorBoom.inputVelocityPidGains(K_PROPORTIONAL_boom, TAU_i_boom, TAU_d_boom, ALPHA_d_boom); //Set Gains
  motorBoom.inputPositionCascadeGain(K_PROPORTIONAL_CASCADE_boom);

  motorArm.initEncoder();
  motorBoom.initEncoder();

  TCCR2B &= ~arduinoPrescalerEraser;
  TCCR2B |= arduinoClockPrescaler;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 011

  SPI.begin();

  nh.initNode(); //start ROS node

  nh.advertise(loop_freq);
  nh.advertise(machine_state_arduino);
  nh.subscribe(logging_sub);
  nh.subscribe(arduino_command);
  //nh.subscribe(power_gradient);
  
  delay(1000);
  motorArm.readEncoder();
  motorBoom.readEncoder();

  motorArm.referencePosition  = motorArm.encoderpos;
  motorBoom.referencePosition = motorBoom.encoderpos;
  pos_initial_arm = motorArm.referencePosition;

  //motorArm.enableMotor();
  //motorBoom.enableMotor();

}

void loop()
{
  motorArm.readEncoder();
  motorBoom.readEncoder();

  motorArm.arduinoReadValuesSpeed();
  motorBoom.arduinoReadValuesSpeed();

  motorArm.arduinoReadValuesCurrent();
  motorBoom.arduinoReadValuesCurrent();

  motorArm.CLC();
  motorBoom.CLC();

  motorArm.arduinoWriteSpeed();
  motorBoom.arduinoWriteSpeed();

  i++;
  nh.spinOnce();
  delay(1);
}

float sign(float value) {
  return float((value > 0) - (value < 0));
}

