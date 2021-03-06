// Generated by gencpp from file simulator/PIDInput.msg
// DO NOT EDIT!


#ifndef SIMULATOR_MESSAGE_PIDINPUT_H
#define SIMULATOR_MESSAGE_PIDINPUT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace simulator
{
template <class ContainerAllocator>
struct PIDInput_
{
  typedef PIDInput_<ContainerAllocator> Type;

  PIDInput_()
    : pid_vel(0.0)
    , pid_error(0.0)  {
    }
  PIDInput_(const ContainerAllocator& _alloc)
    : pid_vel(0.0)
    , pid_error(0.0)  {
  (void)_alloc;
    }



   typedef float _pid_vel_type;
  _pid_vel_type pid_vel;

   typedef float _pid_error_type;
  _pid_error_type pid_error;





  typedef boost::shared_ptr< ::simulator::PIDInput_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::simulator::PIDInput_<ContainerAllocator> const> ConstPtr;

}; // struct PIDInput_

typedef ::simulator::PIDInput_<std::allocator<void> > PIDInput;

typedef boost::shared_ptr< ::simulator::PIDInput > PIDInputPtr;
typedef boost::shared_ptr< ::simulator::PIDInput const> PIDInputConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::simulator::PIDInput_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::simulator::PIDInput_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::simulator::PIDInput_<ContainerAllocator1> & lhs, const ::simulator::PIDInput_<ContainerAllocator2> & rhs)
{
  return lhs.pid_vel == rhs.pid_vel &&
    lhs.pid_error == rhs.pid_error;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::simulator::PIDInput_<ContainerAllocator1> & lhs, const ::simulator::PIDInput_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace simulator

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::simulator::PIDInput_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::simulator::PIDInput_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simulator::PIDInput_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simulator::PIDInput_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simulator::PIDInput_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simulator::PIDInput_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::simulator::PIDInput_<ContainerAllocator> >
{
  static const char* value()
  {
    return "15d51ace2dba29e1b19e1332c9d46c17";
  }

  static const char* value(const ::simulator::PIDInput_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x15d51ace2dba29e1ULL;
  static const uint64_t static_value2 = 0xb19e1332c9d46c17ULL;
};

template<class ContainerAllocator>
struct DataType< ::simulator::PIDInput_<ContainerAllocator> >
{
  static const char* value()
  {
    return "simulator/PIDInput";
  }

  static const char* value(const ::simulator::PIDInput_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::simulator::PIDInput_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 pid_vel\n"
"float32 pid_error\n"
;
  }

  static const char* value(const ::simulator::PIDInput_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::simulator::PIDInput_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pid_vel);
      stream.next(m.pid_error);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PIDInput_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::simulator::PIDInput_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::simulator::PIDInput_<ContainerAllocator>& v)
  {
    s << indent << "pid_vel: ";
    Printer<float>::stream(s, indent + "  ", v.pid_vel);
    s << indent << "pid_error: ";
    Printer<float>::stream(s, indent + "  ", v.pid_error);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SIMULATOR_MESSAGE_PIDINPUT_H
