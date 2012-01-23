/* Auto-generated by genmsg_cpp for file /home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/humanoid_aff_learning/icub_aff_learning/tabletop_2D_segmentation/srv/Perception2D.srv */
#ifndef TABLETOP_2D_SEGMENTATION_SERVICE_PERCEPTION2D_H
#define TABLETOP_2D_SEGMENTATION_SERVICE_PERCEPTION2D_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"



#include "sensor_msgs/Image.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/RegionOfInterest.h"

namespace tabletop_2D_segmentation
{
template <class ContainerAllocator>
struct Perception2DRequest_ {
  typedef Perception2DRequest_<ContainerAllocator> Type;

  Perception2DRequest_()
  : task(0)
  , arg(0)
  , arg2(0)
  {
  }

  Perception2DRequest_(const ContainerAllocator& _alloc)
  : task(0)
  , arg(0)
  , arg2(0)
  {
  }

  typedef uint8_t _task_type;
  uint8_t task;

  typedef int8_t _arg_type;
  int8_t arg;

  typedef int8_t _arg2_type;
  int8_t arg2;

  enum { DO_PERCEPT = 0 };
  enum { EXTRACT_EFFECT = 1 };

private:
  static const char* __s_getDataType_() { return "tabletop_2D_segmentation/Perception2DRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "9135b8f36be2b25d9bf7c242938ee0a0"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "1da76ff735fd425703f5cdeb93da420c"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "uint8 DO_PERCEPT = 0\n\
uint8 EXTRACT_EFFECT = 1\n\
uint8 task\n\
int8 arg\n\
int8 arg2\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, task);
    ros::serialization::serialize(stream, arg);
    ros::serialization::serialize(stream, arg2);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, task);
    ros::serialization::deserialize(stream, arg);
    ros::serialization::deserialize(stream, arg2);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(task);
    size += ros::serialization::serializationLength(arg);
    size += ros::serialization::serializationLength(arg2);
    return size;
  }

  typedef boost::shared_ptr< ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Perception2DRequest
typedef  ::tabletop_2D_segmentation::Perception2DRequest_<std::allocator<void> > Perception2DRequest;

typedef boost::shared_ptr< ::tabletop_2D_segmentation::Perception2DRequest> Perception2DRequestPtr;
typedef boost::shared_ptr< ::tabletop_2D_segmentation::Perception2DRequest const> Perception2DRequestConstPtr;


template <class ContainerAllocator>
struct Perception2DResponse_ {
  typedef Perception2DResponse_<ContainerAllocator> Type;

  Perception2DResponse_()
  : raw_image()
  , filtered_image()
  , roi()
  , face_detected(false)
  , ooi_area(0)
  , ooi_color_r_hist()
  , ooi_color_g_hist()
  , ooi_color_b_hist()
  {
  }

  Perception2DResponse_(const ContainerAllocator& _alloc)
  : raw_image(_alloc)
  , filtered_image(_alloc)
  , roi(_alloc)
  , face_detected(false)
  , ooi_area(0)
  , ooi_color_r_hist(_alloc)
  , ooi_color_g_hist(_alloc)
  , ooi_color_b_hist(_alloc)
  {
  }

  typedef  ::sensor_msgs::Image_<ContainerAllocator>  _raw_image_type;
   ::sensor_msgs::Image_<ContainerAllocator>  raw_image;

  typedef  ::sensor_msgs::Image_<ContainerAllocator>  _filtered_image_type;
   ::sensor_msgs::Image_<ContainerAllocator>  filtered_image;

  typedef  ::sensor_msgs::RegionOfInterest_<ContainerAllocator>  _roi_type;
   ::sensor_msgs::RegionOfInterest_<ContainerAllocator>  roi;

  typedef uint8_t _face_detected_type;
  uint8_t face_detected;

  typedef int16_t _ooi_area_type;
  int16_t ooi_area;

  typedef std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  _ooi_color_r_hist_type;
  std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  ooi_color_r_hist;

  typedef std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  _ooi_color_g_hist_type;
  std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  ooi_color_g_hist;

  typedef std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  _ooi_color_b_hist_type;
  std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  ooi_color_b_hist;


  ROS_DEPRECATED uint32_t get_ooi_color_r_hist_size() const { return (uint32_t)ooi_color_r_hist.size(); }
  ROS_DEPRECATED void set_ooi_color_r_hist_size(uint32_t size) { ooi_color_r_hist.resize((size_t)size); }
  ROS_DEPRECATED void get_ooi_color_r_hist_vec(std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other > & vec) const { vec = this->ooi_color_r_hist; }
  ROS_DEPRECATED void set_ooi_color_r_hist_vec(const std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other > & vec) { this->ooi_color_r_hist = vec; }
  ROS_DEPRECATED uint32_t get_ooi_color_g_hist_size() const { return (uint32_t)ooi_color_g_hist.size(); }
  ROS_DEPRECATED void set_ooi_color_g_hist_size(uint32_t size) { ooi_color_g_hist.resize((size_t)size); }
  ROS_DEPRECATED void get_ooi_color_g_hist_vec(std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other > & vec) const { vec = this->ooi_color_g_hist; }
  ROS_DEPRECATED void set_ooi_color_g_hist_vec(const std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other > & vec) { this->ooi_color_g_hist = vec; }
  ROS_DEPRECATED uint32_t get_ooi_color_b_hist_size() const { return (uint32_t)ooi_color_b_hist.size(); }
  ROS_DEPRECATED void set_ooi_color_b_hist_size(uint32_t size) { ooi_color_b_hist.resize((size_t)size); }
  ROS_DEPRECATED void get_ooi_color_b_hist_vec(std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other > & vec) const { vec = this->ooi_color_b_hist; }
  ROS_DEPRECATED void set_ooi_color_b_hist_vec(const std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other > & vec) { this->ooi_color_b_hist = vec; }
private:
  static const char* __s_getDataType_() { return "tabletop_2D_segmentation/Perception2DResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "91f80513e8b2f01e9a78a80e67ba125c"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "1da76ff735fd425703f5cdeb93da420c"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
\n\
\n\
sensor_msgs/Image raw_image\n\
sensor_msgs/Image filtered_image\n\
sensor_msgs/RegionOfInterest roi\n\
bool face_detected\n\
int16 ooi_area\n\
int8[] ooi_color_r_hist\n\
int8[] ooi_color_g_hist\n\
int8[] ooi_color_b_hist\n\
\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in src/image_encodings.cpp\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: sensor_msgs/RegionOfInterest\n\
# This message is used to specify a region of interest within an image.\n\
#\n\
# When used to specify the ROI setting of the camera when the image was\n\
# taken, the height and width fields should either match the height and\n\
# width fields for the associated image; or height = width = 0\n\
# indicates that the full resolution image was captured.\n\
\n\
uint32 x_offset  # Leftmost pixel of the ROI\n\
                 # (0 if the ROI includes the left edge of the image)\n\
uint32 y_offset  # Topmost pixel of the ROI\n\
                 # (0 if the ROI includes the top edge of the image)\n\
uint32 height    # Height of ROI\n\
uint32 width     # Width of ROI\n\
\n\
# True if a distinct rectified ROI should be calculated from the \"raw\"\n\
# ROI in this message. Typically this should be False if the full image\n\
# is captured (ROI not used), and True if a subwindow is captured (ROI\n\
# used).\n\
bool do_rectify\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, raw_image);
    ros::serialization::serialize(stream, filtered_image);
    ros::serialization::serialize(stream, roi);
    ros::serialization::serialize(stream, face_detected);
    ros::serialization::serialize(stream, ooi_area);
    ros::serialization::serialize(stream, ooi_color_r_hist);
    ros::serialization::serialize(stream, ooi_color_g_hist);
    ros::serialization::serialize(stream, ooi_color_b_hist);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, raw_image);
    ros::serialization::deserialize(stream, filtered_image);
    ros::serialization::deserialize(stream, roi);
    ros::serialization::deserialize(stream, face_detected);
    ros::serialization::deserialize(stream, ooi_area);
    ros::serialization::deserialize(stream, ooi_color_r_hist);
    ros::serialization::deserialize(stream, ooi_color_g_hist);
    ros::serialization::deserialize(stream, ooi_color_b_hist);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(raw_image);
    size += ros::serialization::serializationLength(filtered_image);
    size += ros::serialization::serializationLength(roi);
    size += ros::serialization::serializationLength(face_detected);
    size += ros::serialization::serializationLength(ooi_area);
    size += ros::serialization::serializationLength(ooi_color_r_hist);
    size += ros::serialization::serializationLength(ooi_color_g_hist);
    size += ros::serialization::serializationLength(ooi_color_b_hist);
    return size;
  }

  typedef boost::shared_ptr< ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Perception2DResponse
typedef  ::tabletop_2D_segmentation::Perception2DResponse_<std::allocator<void> > Perception2DResponse;

typedef boost::shared_ptr< ::tabletop_2D_segmentation::Perception2DResponse> Perception2DResponsePtr;
typedef boost::shared_ptr< ::tabletop_2D_segmentation::Perception2DResponse const> Perception2DResponseConstPtr;

struct Perception2D
{

typedef Perception2DRequest Request;
typedef Perception2DResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Perception2D
} // namespace tabletop_2D_segmentation

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9135b8f36be2b25d9bf7c242938ee0a0";
  }

  static const char* value(const  ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9135b8f36be2b25dULL;
  static const uint64_t static_value2 = 0x9bf7c242938ee0a0ULL;
};

template<class ContainerAllocator>
struct DataType< ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tabletop_2D_segmentation/Perception2DRequest";
  }

  static const char* value(const  ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 DO_PERCEPT = 0\n\
uint8 EXTRACT_EFFECT = 1\n\
uint8 task\n\
int8 arg\n\
int8 arg2\n\
\n\
";
  }

  static const char* value(const  ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "91f80513e8b2f01e9a78a80e67ba125c";
  }

  static const char* value(const  ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x91f80513e8b2f01eULL;
  static const uint64_t static_value2 = 0x9a78a80e67ba125cULL;
};

template<class ContainerAllocator>
struct DataType< ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tabletop_2D_segmentation/Perception2DResponse";
  }

  static const char* value(const  ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
sensor_msgs/Image raw_image\n\
sensor_msgs/Image filtered_image\n\
sensor_msgs/RegionOfInterest roi\n\
bool face_detected\n\
int16 ooi_area\n\
int8[] ooi_color_r_hist\n\
int8[] ooi_color_g_hist\n\
int8[] ooi_color_b_hist\n\
\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in src/image_encodings.cpp\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: sensor_msgs/RegionOfInterest\n\
# This message is used to specify a region of interest within an image.\n\
#\n\
# When used to specify the ROI setting of the camera when the image was\n\
# taken, the height and width fields should either match the height and\n\
# width fields for the associated image; or height = width = 0\n\
# indicates that the full resolution image was captured.\n\
\n\
uint32 x_offset  # Leftmost pixel of the ROI\n\
                 # (0 if the ROI includes the left edge of the image)\n\
uint32 y_offset  # Topmost pixel of the ROI\n\
                 # (0 if the ROI includes the top edge of the image)\n\
uint32 height    # Height of ROI\n\
uint32 width     # Width of ROI\n\
\n\
# True if a distinct rectified ROI should be calculated from the \"raw\"\n\
# ROI in this message. Typically this should be False if the full image\n\
# is captured (ROI not used), and True if a subwindow is captured (ROI\n\
# used).\n\
bool do_rectify\n\
\n\
";
  }

  static const char* value(const  ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.task);
    stream.next(m.arg);
    stream.next(m.arg2);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Perception2DRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.raw_image);
    stream.next(m.filtered_image);
    stream.next(m.roi);
    stream.next(m.face_detected);
    stream.next(m.ooi_area);
    stream.next(m.ooi_color_r_hist);
    stream.next(m.ooi_color_g_hist);
    stream.next(m.ooi_color_b_hist);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Perception2DResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<tabletop_2D_segmentation::Perception2D> {
  static const char* value() 
  {
    return "1da76ff735fd425703f5cdeb93da420c";
  }

  static const char* value(const tabletop_2D_segmentation::Perception2D&) { return value(); } 
};

template<>
struct DataType<tabletop_2D_segmentation::Perception2D> {
  static const char* value() 
  {
    return "tabletop_2D_segmentation/Perception2D";
  }

  static const char* value(const tabletop_2D_segmentation::Perception2D&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1da76ff735fd425703f5cdeb93da420c";
  }

  static const char* value(const tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tabletop_2D_segmentation/Perception2D";
  }

  static const char* value(const tabletop_2D_segmentation::Perception2DRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1da76ff735fd425703f5cdeb93da420c";
  }

  static const char* value(const tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tabletop_2D_segmentation/Perception2D";
  }

  static const char* value(const tabletop_2D_segmentation::Perception2DResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // TABLETOP_2D_SEGMENTATION_SERVICE_PERCEPTION2D_H

