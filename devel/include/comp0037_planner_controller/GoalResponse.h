// Generated by gencpp from file comp0037_planner_controller/GoalResponse.msg
// DO NOT EDIT!


#ifndef COMP0037_PLANNER_CONTROLLER_MESSAGE_GOALRESPONSE_H
#define COMP0037_PLANNER_CONTROLLER_MESSAGE_GOALRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace comp0037_planner_controller
{
template <class ContainerAllocator>
struct GoalResponse_
{
  typedef GoalResponse_<ContainerAllocator> Type;

  GoalResponse_()
    : reachedGoal(false)  {
    }
  GoalResponse_(const ContainerAllocator& _alloc)
    : reachedGoal(false)  {
  (void)_alloc;
    }



   typedef uint8_t _reachedGoal_type;
  _reachedGoal_type reachedGoal;





  typedef boost::shared_ptr< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GoalResponse_

typedef ::comp0037_planner_controller::GoalResponse_<std::allocator<void> > GoalResponse;

typedef boost::shared_ptr< ::comp0037_planner_controller::GoalResponse > GoalResponsePtr;
typedef boost::shared_ptr< ::comp0037_planner_controller::GoalResponse const> GoalResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace comp0037_planner_controller

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "81492924865f72c18a6f4bdd91cb99e9";
  }

  static const char* value(const ::comp0037_planner_controller::GoalResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x81492924865f72c1ULL;
  static const uint64_t static_value2 = 0x8a6f4bdd91cb99e9ULL;
};

template<class ContainerAllocator>
struct DataType< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "comp0037_planner_controller/GoalResponse";
  }

  static const char* value(const ::comp0037_planner_controller::GoalResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool reachedGoal\n\
\n\
";
  }

  static const char* value(const ::comp0037_planner_controller::GoalResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.reachedGoal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoalResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::comp0037_planner_controller::GoalResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::comp0037_planner_controller::GoalResponse_<ContainerAllocator>& v)
  {
    s << indent << "reachedGoal: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reachedGoal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COMP0037_PLANNER_CONTROLLER_MESSAGE_GOALRESPONSE_H
