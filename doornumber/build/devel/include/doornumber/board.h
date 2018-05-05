// Generated by gencpp from file doornumber/board.msg
// DO NOT EDIT!


#ifndef DOORNUMBER_MESSAGE_BOARD_H
#define DOORNUMBER_MESSAGE_BOARD_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace doornumber
{
template <class ContainerAllocator>
struct board_
{
  typedef board_<ContainerAllocator> Type;

  board_()
    : tlx(0)
    , tly(0)
    , brx(0)
    , bry(0)
    , text()
    , confidence(0.0)  {
    }
  board_(const ContainerAllocator& _alloc)
    : tlx(0)
    , tly(0)
    , brx(0)
    , bry(0)
    , text(_alloc)
    , confidence(0.0)  {
  (void)_alloc;
    }



   typedef int16_t _tlx_type;
  _tlx_type tlx;

   typedef int16_t _tly_type;
  _tly_type tly;

   typedef int16_t _brx_type;
  _brx_type brx;

   typedef int16_t _bry_type;
  _bry_type bry;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _text_type;
  _text_type text;

   typedef float _confidence_type;
  _confidence_type confidence;




  typedef boost::shared_ptr< ::doornumber::board_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::doornumber::board_<ContainerAllocator> const> ConstPtr;

}; // struct board_

typedef ::doornumber::board_<std::allocator<void> > board;

typedef boost::shared_ptr< ::doornumber::board > boardPtr;
typedef boost::shared_ptr< ::doornumber::board const> boardConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::doornumber::board_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::doornumber::board_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace doornumber

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'doornumber': ['/home/jk/catkin_lzw/src/doornumber/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::doornumber::board_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::doornumber::board_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::doornumber::board_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::doornumber::board_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::doornumber::board_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::doornumber::board_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::doornumber::board_<ContainerAllocator> >
{
  static const char* value()
  {
    return "41a76418e981dca789597de9cfa6bfa8";
  }

  static const char* value(const ::doornumber::board_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x41a76418e981dca7ULL;
  static const uint64_t static_value2 = 0x89597de9cfa6bfa8ULL;
};

template<class ContainerAllocator>
struct DataType< ::doornumber::board_<ContainerAllocator> >
{
  static const char* value()
  {
    return "doornumber/board";
  }

  static const char* value(const ::doornumber::board_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::doornumber::board_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 tlx\n\
int16 tly\n\
int16 brx\n\
int16 bry\n\
string text\n\
float32 confidence\n\
";
  }

  static const char* value(const ::doornumber::board_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::doornumber::board_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tlx);
      stream.next(m.tly);
      stream.next(m.brx);
      stream.next(m.bry);
      stream.next(m.text);
      stream.next(m.confidence);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct board_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::doornumber::board_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::doornumber::board_<ContainerAllocator>& v)
  {
    s << indent << "tlx: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tlx);
    s << indent << "tly: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tly);
    s << indent << "brx: ";
    Printer<int16_t>::stream(s, indent + "  ", v.brx);
    s << indent << "bry: ";
    Printer<int16_t>::stream(s, indent + "  ", v.bry);
    s << indent << "text: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.text);
    s << indent << "confidence: ";
    Printer<float>::stream(s, indent + "  ", v.confidence);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOORNUMBER_MESSAGE_BOARD_H
