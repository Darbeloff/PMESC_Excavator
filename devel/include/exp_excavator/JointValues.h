// Generated by gencpp from file exp_excavator/JointValues.msg
// DO NOT EDIT!


#ifndef EXP_EXCAVATOR_MESSAGE_JOINTVALUES_H
#define EXP_EXCAVATOR_MESSAGE_JOINTVALUES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace exp_excavator
{
template <class ContainerAllocator>
struct JointValues_
{
  typedef JointValues_<ContainerAllocator> Type;

  JointValues_()
    : boom(0.0)
    , arm(0.0)
    , bucket(0.0)
    , swing(0.0)  {
    }
  JointValues_(const ContainerAllocator& _alloc)
    : boom(0.0)
    , arm(0.0)
    , bucket(0.0)
    , swing(0.0)  {
  (void)_alloc;
    }



   typedef double _boom_type;
  _boom_type boom;

   typedef double _arm_type;
  _arm_type arm;

   typedef double _bucket_type;
  _bucket_type bucket;

   typedef double _swing_type;
  _swing_type swing;




  typedef boost::shared_ptr< ::exp_excavator::JointValues_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::exp_excavator::JointValues_<ContainerAllocator> const> ConstPtr;

}; // struct JointValues_

typedef ::exp_excavator::JointValues_<std::allocator<void> > JointValues;

typedef boost::shared_ptr< ::exp_excavator::JointValues > JointValuesPtr;
typedef boost::shared_ptr< ::exp_excavator::JointValues const> JointValuesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::exp_excavator::JointValues_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::exp_excavator::JointValues_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace exp_excavator

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'exp_excavator': ['/home/weitung/excavation_ws/src/exp_excavator/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::exp_excavator::JointValues_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::exp_excavator::JointValues_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::exp_excavator::JointValues_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::exp_excavator::JointValues_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::exp_excavator::JointValues_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::exp_excavator::JointValues_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::exp_excavator::JointValues_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9d95cfbc464254b141f1bf3578f27d92";
  }

  static const char* value(const ::exp_excavator::JointValues_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9d95cfbc464254b1ULL;
  static const uint64_t static_value2 = 0x41f1bf3578f27d92ULL;
};

template<class ContainerAllocator>
struct DataType< ::exp_excavator::JointValues_<ContainerAllocator> >
{
  static const char* value()
  {
    return "exp_excavator/JointValues";
  }

  static const char* value(const ::exp_excavator::JointValues_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::exp_excavator::JointValues_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 boom\n\
float64 arm\n\
float64 bucket\n\
float64 swing\n\
";
  }

  static const char* value(const ::exp_excavator::JointValues_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::exp_excavator::JointValues_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.boom);
      stream.next(m.arm);
      stream.next(m.bucket);
      stream.next(m.swing);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointValues_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::exp_excavator::JointValues_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::exp_excavator::JointValues_<ContainerAllocator>& v)
  {
    s << indent << "boom: ";
    Printer<double>::stream(s, indent + "  ", v.boom);
    s << indent << "arm: ";
    Printer<double>::stream(s, indent + "  ", v.arm);
    s << indent << "bucket: ";
    Printer<double>::stream(s, indent + "  ", v.bucket);
    s << indent << "swing: ";
    Printer<double>::stream(s, indent + "  ", v.swing);
  }
};

} // namespace message_operations
} // namespace ros

#endif // EXP_EXCAVATOR_MESSAGE_JOINTVALUES_H
