// Generated by gencpp from file ros_gpio/CloseGpio.msg
// DO NOT EDIT!


#ifndef ROS_GPIO_MESSAGE_CLOSEGPIO_H
#define ROS_GPIO_MESSAGE_CLOSEGPIO_H

#include <ros/service_traits.h>


#include <ros_gpio/CloseGpioRequest.h>
#include <ros_gpio/CloseGpioResponse.h>


namespace ros_gpio
{

struct CloseGpio
{

typedef CloseGpioRequest Request;
typedef CloseGpioResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct CloseGpio
} // namespace ros_gpio


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ros_gpio::CloseGpio > {
  static const char* value()
  {
    return "09a5a8266af8d6f25a1d421823e44e03";
  }

  static const char* value(const ::ros_gpio::CloseGpio&) { return value(); }
};

template<>
struct DataType< ::ros_gpio::CloseGpio > {
  static const char* value()
  {
    return "ros_gpio/CloseGpio";
  }

  static const char* value(const ::ros_gpio::CloseGpio&) { return value(); }
};


// service_traits::MD5Sum< ::ros_gpio::CloseGpioRequest> should match 
// service_traits::MD5Sum< ::ros_gpio::CloseGpio > 
template<>
struct MD5Sum< ::ros_gpio::CloseGpioRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ros_gpio::CloseGpio >::value();
  }
  static const char* value(const ::ros_gpio::CloseGpioRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_gpio::CloseGpioRequest> should match 
// service_traits::DataType< ::ros_gpio::CloseGpio > 
template<>
struct DataType< ::ros_gpio::CloseGpioRequest>
{
  static const char* value()
  {
    return DataType< ::ros_gpio::CloseGpio >::value();
  }
  static const char* value(const ::ros_gpio::CloseGpioRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ros_gpio::CloseGpioResponse> should match 
// service_traits::MD5Sum< ::ros_gpio::CloseGpio > 
template<>
struct MD5Sum< ::ros_gpio::CloseGpioResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ros_gpio::CloseGpio >::value();
  }
  static const char* value(const ::ros_gpio::CloseGpioResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_gpio::CloseGpioResponse> should match 
// service_traits::DataType< ::ros_gpio::CloseGpio > 
template<>
struct DataType< ::ros_gpio::CloseGpioResponse>
{
  static const char* value()
  {
    return DataType< ::ros_gpio::CloseGpio >::value();
  }
  static const char* value(const ::ros_gpio::CloseGpioResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROS_GPIO_MESSAGE_CLOSEGPIO_H
