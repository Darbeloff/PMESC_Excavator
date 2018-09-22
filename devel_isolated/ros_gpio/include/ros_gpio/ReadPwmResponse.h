// Generated by gencpp from file ros_gpio/ReadPwmResponse.msg
// DO NOT EDIT!


#ifndef ROS_GPIO_MESSAGE_READPWMRESPONSE_H
#define ROS_GPIO_MESSAGE_READPWMRESPONSE_H


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
struct ReadPwmResponse_
{
  typedef ReadPwmResponse_<ContainerAllocator> Type;

  ReadPwmResponse_()
    : percent(0.0)  {
    }
  ReadPwmResponse_(const ContainerAllocator& _alloc)
    : percent(0.0)  {
  (void)_alloc;
    }



   typedef float _percent_type;
  _percent_type percent;




  typedef boost::shared_ptr< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ReadPwmResponse_

typedef ::ros_gpio::ReadPwmResponse_<std::allocator<void> > ReadPwmResponse;

typedef boost::shared_ptr< ::ros_gpio::ReadPwmResponse > ReadPwmResponsePtr;
typedef boost::shared_ptr< ::ros_gpio::ReadPwmResponse const> ReadPwmResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_gpio::ReadPwmResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d8ec0a0f9e4c365f68112b4b8ef1808d";
  }

  static const char* value(const ::ros_gpio::ReadPwmResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd8ec0a0f9e4c365fULL;
  static const uint64_t static_value2 = 0x68112b4b8ef1808dULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_gpio/ReadPwmResponse";
  }

  static const char* value(const ::ros_gpio::ReadPwmResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 percent\n\
\n\
";
  }

  static const char* value(const ::ros_gpio::ReadPwmResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.percent);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ReadPwmResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_gpio::ReadPwmResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_gpio::ReadPwmResponse_<ContainerAllocator>& v)
  {
    s << indent << "percent: ";
    Printer<float>::stream(s, indent + "  ", v.percent);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_GPIO_MESSAGE_READPWMRESPONSE_H