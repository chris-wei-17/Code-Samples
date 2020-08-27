// Generated by gencpp from file zed_wrapper/stop_3d_mappingRequest.msg
// DO NOT EDIT!


#ifndef ZED_WRAPPER_MESSAGE_STOP_3D_MAPPINGREQUEST_H
#define ZED_WRAPPER_MESSAGE_STOP_3D_MAPPINGREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace zed_wrapper
{
template <class ContainerAllocator>
struct stop_3d_mappingRequest_
{
  typedef stop_3d_mappingRequest_<ContainerAllocator> Type;

  stop_3d_mappingRequest_()
    {
    }
  stop_3d_mappingRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> const> ConstPtr;

}; // struct stop_3d_mappingRequest_

typedef ::zed_wrapper::stop_3d_mappingRequest_<std::allocator<void> > stop_3d_mappingRequest;

typedef boost::shared_ptr< ::zed_wrapper::stop_3d_mappingRequest > stop_3d_mappingRequestPtr;
typedef boost::shared_ptr< ::zed_wrapper::stop_3d_mappingRequest const> stop_3d_mappingRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace zed_wrapper

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'zed_wrapper': ['/home/jetson1/Racecar/racecar_ws/src/zed-ros-wrapper/zed_wrapper/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "zed_wrapper/stop_3d_mappingRequest";
  }

  static const char* value(const ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
"\n"
;
  }

  static const char* value(const ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct stop_3d_mappingRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::zed_wrapper::stop_3d_mappingRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // ZED_WRAPPER_MESSAGE_STOP_3D_MAPPINGREQUEST_H
