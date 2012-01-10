; Auto-generated. Do not edit!


(cl:in-package feature_factory-srv)


;//! \htmlinclude FeatureCalculation-request.msg.html

(cl:defclass <FeatureCalculation-request> (roslisp-msg-protocol:ros-message)
  ((calc_amplitude
    :reader calc_amplitude
    :initarg :calc_amplitude
    :type cl:boolean
    :initform cl:nil)
   (calc_confidence
    :reader calc_confidence
    :initarg :calc_confidence
    :type cl:boolean
    :initform cl:nil)
   (calc_distance
    :reader calc_distance
    :initarg :calc_distance
    :type cl:boolean
    :initform cl:nil)
   (calc_normals_azi
    :reader calc_normals_azi
    :initarg :calc_normals_azi
    :type cl:boolean
    :initform cl:nil)
   (calc_normals_zen
    :reader calc_normals_zen
    :initarg :calc_normals_zen
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FeatureCalculation-request (<FeatureCalculation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FeatureCalculation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FeatureCalculation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feature_factory-srv:<FeatureCalculation-request> is deprecated: use feature_factory-srv:FeatureCalculation-request instead.")))

(cl:ensure-generic-function 'calc_amplitude-val :lambda-list '(m))
(cl:defmethod calc_amplitude-val ((m <FeatureCalculation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_factory-srv:calc_amplitude-val is deprecated.  Use feature_factory-srv:calc_amplitude instead.")
  (calc_amplitude m))

(cl:ensure-generic-function 'calc_confidence-val :lambda-list '(m))
(cl:defmethod calc_confidence-val ((m <FeatureCalculation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_factory-srv:calc_confidence-val is deprecated.  Use feature_factory-srv:calc_confidence instead.")
  (calc_confidence m))

(cl:ensure-generic-function 'calc_distance-val :lambda-list '(m))
(cl:defmethod calc_distance-val ((m <FeatureCalculation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_factory-srv:calc_distance-val is deprecated.  Use feature_factory-srv:calc_distance instead.")
  (calc_distance m))

(cl:ensure-generic-function 'calc_normals_azi-val :lambda-list '(m))
(cl:defmethod calc_normals_azi-val ((m <FeatureCalculation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_factory-srv:calc_normals_azi-val is deprecated.  Use feature_factory-srv:calc_normals_azi instead.")
  (calc_normals_azi m))

(cl:ensure-generic-function 'calc_normals_zen-val :lambda-list '(m))
(cl:defmethod calc_normals_zen-val ((m <FeatureCalculation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_factory-srv:calc_normals_zen-val is deprecated.  Use feature_factory-srv:calc_normals_zen instead.")
  (calc_normals_zen m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FeatureCalculation-request>) ostream)
  "Serializes a message object of type '<FeatureCalculation-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'calc_amplitude) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'calc_confidence) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'calc_distance) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'calc_normals_azi) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'calc_normals_zen) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FeatureCalculation-request>) istream)
  "Deserializes a message object of type '<FeatureCalculation-request>"
    (cl:setf (cl:slot-value msg 'calc_amplitude) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'calc_confidence) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'calc_distance) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'calc_normals_azi) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'calc_normals_zen) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FeatureCalculation-request>)))
  "Returns string type for a service object of type '<FeatureCalculation-request>"
  "feature_factory/FeatureCalculationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FeatureCalculation-request)))
  "Returns string type for a service object of type 'FeatureCalculation-request"
  "feature_factory/FeatureCalculationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FeatureCalculation-request>)))
  "Returns md5sum for a message object of type '<FeatureCalculation-request>"
  "d90c5348877c0b5a172d453461340109")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FeatureCalculation-request)))
  "Returns md5sum for a message object of type 'FeatureCalculation-request"
  "d90c5348877c0b5a172d453461340109")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FeatureCalculation-request>)))
  "Returns full string definition for message of type '<FeatureCalculation-request>"
  (cl:format cl:nil "bool calc_amplitude~%bool calc_confidence~%bool calc_distance~%bool calc_normals_azi~%bool calc_normals_zen~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FeatureCalculation-request)))
  "Returns full string definition for message of type 'FeatureCalculation-request"
  (cl:format cl:nil "bool calc_amplitude~%bool calc_confidence~%bool calc_distance~%bool calc_normals_azi~%bool calc_normals_zen~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FeatureCalculation-request>))
  (cl:+ 0
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FeatureCalculation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FeatureCalculation-request
    (cl:cons ':calc_amplitude (calc_amplitude msg))
    (cl:cons ':calc_confidence (calc_confidence msg))
    (cl:cons ':calc_distance (calc_distance msg))
    (cl:cons ':calc_normals_azi (calc_normals_azi msg))
    (cl:cons ':calc_normals_zen (calc_normals_zen msg))
))
;//! \htmlinclude FeatureCalculation-response.msg.html

(cl:defclass <FeatureCalculation-response> (roslisp-msg-protocol:ros-message)
  ((f_distances
    :reader f_distances
    :initarg :f_distances
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (f_confidences
    :reader f_confidences
    :initarg :f_confidences
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (f_amplitudes
    :reader f_amplitudes
    :initarg :f_amplitudes
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (f_normals_azi
    :reader f_normals_azi
    :initarg :f_normals_azi
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (f_normals_zen
    :reader f_normals_zen
    :initarg :f_normals_zen
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass FeatureCalculation-response (<FeatureCalculation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FeatureCalculation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FeatureCalculation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feature_factory-srv:<FeatureCalculation-response> is deprecated: use feature_factory-srv:FeatureCalculation-response instead.")))

(cl:ensure-generic-function 'f_distances-val :lambda-list '(m))
(cl:defmethod f_distances-val ((m <FeatureCalculation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_factory-srv:f_distances-val is deprecated.  Use feature_factory-srv:f_distances instead.")
  (f_distances m))

(cl:ensure-generic-function 'f_confidences-val :lambda-list '(m))
(cl:defmethod f_confidences-val ((m <FeatureCalculation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_factory-srv:f_confidences-val is deprecated.  Use feature_factory-srv:f_confidences instead.")
  (f_confidences m))

(cl:ensure-generic-function 'f_amplitudes-val :lambda-list '(m))
(cl:defmethod f_amplitudes-val ((m <FeatureCalculation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_factory-srv:f_amplitudes-val is deprecated.  Use feature_factory-srv:f_amplitudes instead.")
  (f_amplitudes m))

(cl:ensure-generic-function 'f_normals_azi-val :lambda-list '(m))
(cl:defmethod f_normals_azi-val ((m <FeatureCalculation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_factory-srv:f_normals_azi-val is deprecated.  Use feature_factory-srv:f_normals_azi instead.")
  (f_normals_azi m))

(cl:ensure-generic-function 'f_normals_zen-val :lambda-list '(m))
(cl:defmethod f_normals_zen-val ((m <FeatureCalculation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_factory-srv:f_normals_zen-val is deprecated.  Use feature_factory-srv:f_normals_zen instead.")
  (f_normals_zen m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FeatureCalculation-response>) ostream)
  "Serializes a message object of type '<FeatureCalculation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'f_distances) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'f_confidences) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'f_amplitudes) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'f_normals_azi) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'f_normals_zen) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FeatureCalculation-response>) istream)
  "Deserializes a message object of type '<FeatureCalculation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'f_distances) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'f_confidences) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'f_amplitudes) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'f_normals_azi) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'f_normals_zen) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FeatureCalculation-response>)))
  "Returns string type for a service object of type '<FeatureCalculation-response>"
  "feature_factory/FeatureCalculationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FeatureCalculation-response)))
  "Returns string type for a service object of type 'FeatureCalculation-response"
  "feature_factory/FeatureCalculationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FeatureCalculation-response>)))
  "Returns md5sum for a message object of type '<FeatureCalculation-response>"
  "d90c5348877c0b5a172d453461340109")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FeatureCalculation-response)))
  "Returns md5sum for a message object of type 'FeatureCalculation-response"
  "d90c5348877c0b5a172d453461340109")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FeatureCalculation-response>)))
  "Returns full string definition for message of type '<FeatureCalculation-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 f_distances~%sensor_msgs/PointCloud2 f_confidences~%sensor_msgs/PointCloud2 f_amplitudes~%sensor_msgs/PointCloud2 f_normals_azi~%sensor_msgs/PointCloud2 f_normals_zen~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FeatureCalculation-response)))
  "Returns full string definition for message of type 'FeatureCalculation-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2 f_distances~%sensor_msgs/PointCloud2 f_confidences~%sensor_msgs/PointCloud2 f_amplitudes~%sensor_msgs/PointCloud2 f_normals_azi~%sensor_msgs/PointCloud2 f_normals_zen~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FeatureCalculation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'f_distances))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'f_confidences))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'f_amplitudes))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'f_normals_azi))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'f_normals_zen))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FeatureCalculation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FeatureCalculation-response
    (cl:cons ':f_distances (f_distances msg))
    (cl:cons ':f_confidences (f_confidences msg))
    (cl:cons ':f_amplitudes (f_amplitudes msg))
    (cl:cons ':f_normals_azi (f_normals_azi msg))
    (cl:cons ':f_normals_zen (f_normals_zen msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FeatureCalculation)))
  'FeatureCalculation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FeatureCalculation)))
  'FeatureCalculation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FeatureCalculation)))
  "Returns string type for a service object of type '<FeatureCalculation>"
  "feature_factory/FeatureCalculation")