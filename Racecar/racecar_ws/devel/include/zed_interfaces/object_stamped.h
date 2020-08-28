// Generated by gencpp from file zed_interfaces/object_stamped.msg
// DO NOT EDIT!


#ifndef ZED_INTERFACES_MESSAGE_OBJECT_STAMPED_H
#define ZED_INTERFACES_MESSAGE_OBJECT_STAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point32.h>

namespace zed_interfaces
{
template <class ContainerAllocator>
struct object_stamped_
{
  typedef object_stamped_<ContainerAllocator> Type;

  object_stamped_()
    : header()
    , label()
    , label_id(0)
    , confidence(0.0)
    , position()
    , linear_vel()
    , tracking_state(0)
    , bbox_2d()
    , bbox_3d()  {
    }
  object_stamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , label(_alloc)
    , label_id(0)
    , confidence(0.0)
    , position(_alloc)
    , linear_vel(_alloc)
    , tracking_state(0)
    , bbox_2d()
    , bbox_3d()  {
  (void)_alloc;
      bbox_2d.assign( ::geometry_msgs::Point32_<ContainerAllocator> (_alloc));

      bbox_3d.assign( ::geometry_msgs::Point32_<ContainerAllocator> (_alloc));
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _label_type;
  _label_type label;

   typedef int16_t _label_id_type;
  _label_id_type label_id;

   typedef float _confidence_type;
  _confidence_type confidence;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_vel_type;
  _linear_vel_type linear_vel;

   typedef int8_t _tracking_state_type;
  _tracking_state_type tracking_state;

   typedef boost::array< ::geometry_msgs::Point32_<ContainerAllocator> , 4>  _bbox_2d_type;
  _bbox_2d_type bbox_2d;

   typedef boost::array< ::geometry_msgs::Point32_<ContainerAllocator> , 8>  _bbox_3d_type;
  _bbox_3d_type bbox_3d;





  typedef boost::shared_ptr< ::zed_interfaces::object_stamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::zed_interfaces::object_stamped_<ContainerAllocator> const> ConstPtr;

}; // struct object_stamped_

typedef ::zed_interfaces::object_stamped_<std::allocator<void> > object_stamped;

typedef boost::shared_ptr< ::zed_interfaces::object_stamped > object_stampedPtr;
typedef boost::shared_ptr< ::zed_interfaces::object_stamped const> object_stampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::zed_interfaces::object_stamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::zed_interfaces::object_stamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace zed_interfaces

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'zed_interfaces': ['/home/jetson1/Racecar/racecar_ws/src/zed-ros-wrapper/zed_interfaces/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::zed_interfaces::object_stamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::zed_interfaces::object_stamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::zed_interfaces::object_stamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::zed_interfaces::object_stamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::zed_interfaces::object_stamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::zed_interfaces::object_stamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::zed_interfaces::object_stamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d424656ac6d81bea041e4eb6457725ad";
  }

  static const char* value(const ::zed_interfaces::object_stamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd424656ac6d81beaULL;
  static const uint64_t static_value2 = 0x041e4eb6457725adULL;
};

template<class ContainerAllocator>
struct DataType< ::zed_interfaces::object_stamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "zed_interfaces/object_stamped";
  }

  static const char* value(const ::zed_interfaces::object_stamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::zed_interfaces::object_stamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Standard Header\n"
"Header header\n"
"\n"
"# Object label\n"
"string label\n"
"\n"
"# Object label ID\n"
"int16 label_id\n"
"\n"
"# Object confidence level (1-99)\n"
"float32 confidence\n"
"\n"
"# Object centroid\n"
"geometry_msgs/Point32 position\n"
"\n"
"# Object velocity\n"
"geometry_msgs/Vector3 linear_vel\n"
"\n"
"# Tracking state\n"
"# 0 -> OFF (object not valid)\n"
"# 1 -> OK\n"
"# 2 -> SEARCHING (occlusion occurred, trajectory is estimated)\n"
"int8 tracking_state \n"
"\n"
"# 2D Bounding box projected to Camera image\n"
"#      0 ------- 1\n"
"#      |         |\n"
"#      |         |\n"
"#      |         |\n"
"#      3 ------- 2\n"
"geometry_msgs/Point32[4] bbox_2d\n"
"\n"
"# 3D Bounding box in world frame\n"
"#      1 ------- 2\n"
"#     /.        /|\n"
"#    0 ------- 3 |\n"
"#    | .       | |           \n"
"#    | 5.......| 6\n"
"#    |.        |/       \n"
"#    4 ------- 7\n"
"geometry_msgs/Point32[8] bbox_3d\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::zed_interfaces::object_stamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::zed_interfaces::object_stamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.label);
      stream.next(m.label_id);
      stream.next(m.confidence);
      stream.next(m.position);
      stream.next(m.linear_vel);
      stream.next(m.tracking_state);
      stream.next(m.bbox_2d);
      stream.next(m.bbox_3d);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct object_stamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::zed_interfaces::object_stamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::zed_interfaces::object_stamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "label: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.label);
    s << indent << "label_id: ";
    Printer<int16_t>::stream(s, indent + "  ", v.label_id);
    s << indent << "confidence: ";
    Printer<float>::stream(s, indent + "  ", v.confidence);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "linear_vel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.linear_vel);
    s << indent << "tracking_state: ";
    Printer<int8_t>::stream(s, indent + "  ", v.tracking_state);
    s << indent << "bbox_2d[]" << std::endl;
    for (size_t i = 0; i < v.bbox_2d.size(); ++i)
    {
      s << indent << "  bbox_2d[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "    ", v.bbox_2d[i]);
    }
    s << indent << "bbox_3d[]" << std::endl;
    for (size_t i = 0; i < v.bbox_3d.size(); ++i)
    {
      s << indent << "  bbox_3d[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "    ", v.bbox_3d[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ZED_INTERFACES_MESSAGE_OBJECT_STAMPED_H