// Generated by gencpp from file zed_interfaces/start_remote_streamRequest.msg
// DO NOT EDIT!


#ifndef ZED_INTERFACES_MESSAGE_START_REMOTE_STREAMREQUEST_H
#define ZED_INTERFACES_MESSAGE_START_REMOTE_STREAMREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace zed_interfaces
{
template <class ContainerAllocator>
struct start_remote_streamRequest_
{
  typedef start_remote_streamRequest_<ContainerAllocator> Type;

  start_remote_streamRequest_()
    {
    }
  start_remote_streamRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(codec)
  #undef codec
#endif
#if defined(_WIN32) && defined(port)
  #undef port
#endif
#if defined(_WIN32) && defined(bitrate)
  #undef bitrate
#endif
#if defined(_WIN32) && defined(gop_size)
  #undef gop_size
#endif
#if defined(_WIN32) && defined(adaptative_bitrate)
  #undef adaptative_bitrate
#endif

  enum {
    codec = 0u,
    port = 30000u,
    bitrate = 2000u,
    gop_size = -1,
  };

  static const uint8_t adaptative_bitrate;

  typedef boost::shared_ptr< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> const> ConstPtr;

}; // struct start_remote_streamRequest_

typedef ::zed_interfaces::start_remote_streamRequest_<std::allocator<void> > start_remote_streamRequest;

typedef boost::shared_ptr< ::zed_interfaces::start_remote_streamRequest > start_remote_streamRequestPtr;
typedef boost::shared_ptr< ::zed_interfaces::start_remote_streamRequest const> start_remote_streamRequestConstPtr;

// constants requiring out of line definition

   

   

   

   

   
   template<typename ContainerAllocator> const uint8_t
      start_remote_streamRequest_<ContainerAllocator>::adaptative_bitrate =
        
           0
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace zed_interfaces

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a6f55a6077162992b395e1b483a03367";
  }

  static const char* value(const ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa6f55a6077162992ULL;
  static const uint64_t static_value2 = 0xb395e1b483a03367ULL;
};

template<class ContainerAllocator>
struct DataType< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "zed_interfaces/start_remote_streamRequest";
  }

  static const char* value(const ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
"uint8 codec=0\n"
"\n"
"\n"
"\n"
"uint16 port=30000\n"
"\n"
"\n"
"uint32 bitrate=2000\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"int32 gop_size=-1\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"bool adaptative_bitrate=False\n"
;
  }

  static const char* value(const ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct start_remote_streamRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::zed_interfaces::start_remote_streamRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // ZED_INTERFACES_MESSAGE_START_REMOTE_STREAMREQUEST_H
