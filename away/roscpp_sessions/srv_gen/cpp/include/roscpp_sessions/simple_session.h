/* Auto-generated by genmsg_cpp for file /home/tribot/ros/opentribot/roscpp_sessions/srv/simple_session.srv */
#ifndef ROSCPP_SESSIONS_SERVICE_SIMPLE_SESSION_H
#define ROSCPP_SESSIONS_SERVICE_SIMPLE_SESSION_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"




namespace roscpp_sessions
{
template <class ContainerAllocator>
struct simple_sessionRequest_ : public ros::Message
{
  typedef simple_sessionRequest_<ContainerAllocator> Type;

  simple_sessionRequest_()
  : sessionid(0)
  , options(0)
  {
  }

  simple_sessionRequest_(const ContainerAllocator& _alloc)
  : sessionid(0)
  , options(0)
  {
  }

  typedef int32_t _sessionid_type;
  int32_t sessionid;

  typedef int32_t _options_type;
  int32_t options;


private:
  static const char* __s_getDataType_() { return "roscpp_sessions/simple_sessionRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "356582ab258a918ca4a809d930432d02"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "42123af46840487b5635bddfd445b0b1"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 sessionid\n\
int32 options\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, sessionid);
    ros::serialization::serialize(stream, options);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, sessionid);
    ros::serialization::deserialize(stream, options);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(sessionid);
    size += ros::serialization::serializationLength(options);
    return size;
  }

  typedef boost::shared_ptr< ::roscpp_sessions::simple_sessionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_sessions::simple_sessionRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct simple_sessionRequest
typedef  ::roscpp_sessions::simple_sessionRequest_<std::allocator<void> > simple_sessionRequest;

typedef boost::shared_ptr< ::roscpp_sessions::simple_sessionRequest> simple_sessionRequestPtr;
typedef boost::shared_ptr< ::roscpp_sessions::simple_sessionRequest const> simple_sessionRequestConstPtr;


template <class ContainerAllocator>
struct simple_sessionResponse_ : public ros::Message
{
  typedef simple_sessionResponse_<ContainerAllocator> Type;

  simple_sessionResponse_()
  : sessionid(0)
  {
  }

  simple_sessionResponse_(const ContainerAllocator& _alloc)
  : sessionid(0)
  {
  }

  typedef int32_t _sessionid_type;
  int32_t sessionid;


private:
  static const char* __s_getDataType_() { return "roscpp_sessions/simple_sessionResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "8e5dc427c491661ad151bd29bea23253"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "42123af46840487b5635bddfd445b0b1"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 sessionid\n\
\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, sessionid);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, sessionid);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(sessionid);
    return size;
  }

  typedef boost::shared_ptr< ::roscpp_sessions::simple_sessionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_sessions::simple_sessionResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct simple_sessionResponse
typedef  ::roscpp_sessions::simple_sessionResponse_<std::allocator<void> > simple_sessionResponse;

typedef boost::shared_ptr< ::roscpp_sessions::simple_sessionResponse> simple_sessionResponsePtr;
typedef boost::shared_ptr< ::roscpp_sessions::simple_sessionResponse const> simple_sessionResponseConstPtr;

struct simple_session
{

typedef simple_sessionRequest Request;
typedef simple_sessionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simple_session
} // namespace roscpp_sessions

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::roscpp_sessions::simple_sessionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "356582ab258a918ca4a809d930432d02";
  }

  static const char* value(const  ::roscpp_sessions::simple_sessionRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x356582ab258a918cULL;
  static const uint64_t static_value2 = 0xa4a809d930432d02ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_sessions::simple_sessionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "roscpp_sessions/simple_sessionRequest";
  }

  static const char* value(const  ::roscpp_sessions::simple_sessionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::roscpp_sessions::simple_sessionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 sessionid\n\
int32 options\n\
\n\
";
  }

  static const char* value(const  ::roscpp_sessions::simple_sessionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::roscpp_sessions::simple_sessionRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::roscpp_sessions::simple_sessionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8e5dc427c491661ad151bd29bea23253";
  }

  static const char* value(const  ::roscpp_sessions::simple_sessionResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8e5dc427c491661aULL;
  static const uint64_t static_value2 = 0xd151bd29bea23253ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_sessions::simple_sessionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "roscpp_sessions/simple_sessionResponse";
  }

  static const char* value(const  ::roscpp_sessions::simple_sessionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::roscpp_sessions::simple_sessionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 sessionid\n\
\n\
\n\
";
  }

  static const char* value(const  ::roscpp_sessions::simple_sessionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::roscpp_sessions::simple_sessionResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::roscpp_sessions::simple_sessionRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.sessionid);
    stream.next(m.options);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simple_sessionRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::roscpp_sessions::simple_sessionResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.sessionid);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simple_sessionResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<roscpp_sessions::simple_session> {
  static const char* value() 
  {
    return "42123af46840487b5635bddfd445b0b1";
  }

  static const char* value(const roscpp_sessions::simple_session&) { return value(); } 
};

template<>
struct DataType<roscpp_sessions::simple_session> {
  static const char* value() 
  {
    return "roscpp_sessions/simple_session";
  }

  static const char* value(const roscpp_sessions::simple_session&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<roscpp_sessions::simple_sessionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "42123af46840487b5635bddfd445b0b1";
  }

  static const char* value(const roscpp_sessions::simple_sessionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<roscpp_sessions::simple_sessionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "roscpp_sessions/simple_session";
  }

  static const char* value(const roscpp_sessions::simple_sessionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<roscpp_sessions::simple_sessionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "42123af46840487b5635bddfd445b0b1";
  }

  static const char* value(const roscpp_sessions::simple_sessionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<roscpp_sessions::simple_sessionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "roscpp_sessions/simple_session";
  }

  static const char* value(const roscpp_sessions::simple_sessionResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ROSCPP_SESSIONS_SERVICE_SIMPLE_SESSION_H
