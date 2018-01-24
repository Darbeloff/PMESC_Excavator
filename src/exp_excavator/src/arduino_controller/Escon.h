#ifndef ESCON_H
#define ESCON_H
//Motor Structure

//PINS ESCON
const int ENABLE_PIN_ARM     = 22;
const int ENABLE_PIN_BOOM    = 24;

const int DIRECTION_PIN_ARM  = 23;
const int DIRECTION_PIN_BOOM = 25;

const int SLAVE_SELECT_ENCODER_PIN_ARM = 53;
const int SLAVE_SELECT_ENCODER_PIN_BOOM = 9;

const int ANALOG_SPEED_PIN_ARM  = 1;
const int ANALOG_SPEED_PIN_BOOM = 2;

const int PWM_CURRENT_COMMAND_PIN_ARM  = 10;
const int PWM_CURRENT_COMMAND_PIN_BOOM = 11;

const float BITS_TO_RAD_PER_SEC = 1.0;
const float RAD_PER_SEC_OFFSET = 1.0;

const int ANALOG_OUT_BITS     = 255;
const float ESCON_PWM_RANGE   = 0.8;
const float ESCON_PWM_MIN     = 0.1;
const float ESCON_PWM_MAX     = 0.9;
const float ESCON_CURRENT_MAX = 5.0;

//Staff constants
//const int   FREQ = 2000;
//const float PERIOD = 0.0005;
//const float PERIOD_MICROS = (PERIOD * 1000000.0);
//const int   SERIAL_FREQ = 100;  // Should be at least 10 times greater than sin wave frequency
//const float SERIAL_PERIOD = 0.01;
//const float SERIAL_PERIOD_MICROS = 10000.0;

//Staff functions
void stopIfFault(void);
void DoSerialSend(void);

#endif //HELPER_H
