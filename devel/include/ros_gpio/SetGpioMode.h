// Generated by gencpp from file ros_gpio/SetGpioMode.msg
// DO NOT EDIT!


#ifndef ROS_GPIO_MESSAGE_SETGPIOMODE_H
#define ROS_GPIO_MESSAGE_SETGPIOMODE_H

#include <ros/service_traits.h>


#include <ros_gpio/SetGpioModeRequest.h>
#include <ros_gpio/SetGpioModeResponse.h>


namespace ros_gpio
{

struct SetGpioMode
{

typedef SetGpioModeRequest Request;
typedef SetGpioModeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetGpioMode
} // namespace ros_gpio


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ros_gpio::SetGpioMode > {
  static const char* value()
  {
    return "1ff581ae789deaa3ee98533ea6dd13d6";
  }

  static const char* value(const ::ros_gpio::SetGpioMode&) { return value(); }
};

template<>
struct DataType< ::ros_gpio::SetGpioMode > {
  static const char* value()
  {
    return "ros_gpio/SetGpioMode";
  }

  static const char* value(const ::ros_gpio::SetGpioMode&) { return value(); }
};


// service_traits::MD5Sum< ::ros_gpio::SetGpioModeRequest> should match 
// service_traits::MD5Sum< ::ros_gpio::SetGpioMode > 
template<>
struct MD5Sum< ::ros_gpio::SetGpioModeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ros_gpio::SetGpioMode >::value();
  }
  static const char* value(const ::ros_gpio::SetGpioModeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_gpio::SetGpioModeRequest> should match 
// service_traits::DataType< ::ros_gpio::SetGpioMode > 
template<>
struct DataType< ::ros_gpio::SetGpioModeRequest>
{
  static const char* value()
  {
    return DataType< ::ros_gpio::SetGpioMode >::value();
  }
  static const char* value(const ::ros_gpio::SetGpioModeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ros_gpio::SetGpioModeResponse> should match 
// service_traits::MD5Sum< ::ros_gpio::SetGpioMode > 
template<>
struct MD5Sum< ::ros_gpio::SetGpioModeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ros_gpio::SetGpioMode >::value();
  }
  static const char* value(const ::ros_gpio::SetGpioModeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_gpio::SetGpioModeResponse> should match 
// service_traits::DataType< ::ros_gpio::SetGpioMode > 
template<>
struct DataType< ::ros_gpio::SetGpioModeResponse>
{
  static const char* value()
  {
    return DataType< ::ros_gpio::SetGpioMode >::value();
  }
  static const char* value(const ::ros_gpio::SetGpioModeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROS_GPIO_MESSAGE_SETGPIOMODE_H
