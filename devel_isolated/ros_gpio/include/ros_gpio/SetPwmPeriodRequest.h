// Generated by gencpp from file ros_gpio/SetPwmPeriodRequest.msg
// DO NOT EDIT!


#ifndef ROS_GPIO_MESSAGE_SETPWMPERIODREQUEST_H
#define ROS_GPIO_MESSAGE_SETPWMPERIODREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ros_gpio
{
template <class ContainerAllocator>
struct SetPwmPeriodRequest_
{
  typedef SetPwmPeriodRequest_<ContainerAllocator> Type;

  SetPwmPeriodRequest_()
    : pin(0)
    , us(0.0)  {
    }
  SetPwmPeriodRequest_(const ContainerAllocator& _alloc)
    : pin(0)
    , us(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _pin_type;
  _pin_type pin;

   typedef float _us_type;
  _us_type us;




  typedef boost::shared_ptr< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetPwmPeriodRequest_

typedef ::ros_gpio::SetPwmPeriodRequest_<std::allocator<void> > SetPwmPeriodRequest;

typedef boost::shared_ptr< ::ros_gpio::SetPwmPeriodRequest > SetPwmPeriodRequestPtr;
typedef boost::shared_ptr< ::ros_gpio::SetPwmPeriodRequest const> SetPwmPeriodRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ros_gpio

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ros_gpio': ['/home/yutak/ros_ws/src/ros_gpio/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "27b52d1e6fba09ca0a2df1a9ffa94455";
  }

  static const char* value(const ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x27b52d1e6fba09caULL;
  static const uint64_t static_value2 = 0x0a2df1a9ffa94455ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_gpio/SetPwmPeriodRequest";
  }

  static const char* value(const ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 pin\n\
float32 us\n\
";
  }

  static const char* value(const ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pin);
      stream.next(m.us);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetPwmPeriodRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_gpio::SetPwmPeriodRequest_<ContainerAllocator>& v)
  {
    s << indent << "pin: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pin);
    s << indent << "us: ";
    Printer<float>::stream(s, indent + "  ", v.us);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_GPIO_MESSAGE_SETPWMPERIODREQUEST_H
