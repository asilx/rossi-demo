; Auto-generated. Do not edit!


(cl:in-package tabletop_2D_segmentation-srv)


;//! \htmlinclude Perception2D-request.msg.html

(cl:defclass <Perception2D-request> (roslisp-msg-protocol:ros-message)
  ((task
    :reader task
    :initarg :task
    :type cl:fixnum
    :initform 0)
   (arg
    :reader arg
    :initarg :arg
    :type cl:fixnum
    :initform 0)
   (arg2
    :reader arg2
    :initarg :arg2
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Perception2D-request (<Perception2D-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Perception2D-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Perception2D-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tabletop_2D_segmentation-srv:<Perception2D-request> is deprecated: use tabletop_2D_segmentation-srv:Perception2D-request instead.")))

(cl:ensure-generic-function 'task-val :lambda-list '(m))
(cl:defmethod task-val ((m <Perception2D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:task-val is deprecated.  Use tabletop_2D_segmentation-srv:task instead.")
  (task m))

(cl:ensure-generic-function 'arg-val :lambda-list '(m))
(cl:defmethod arg-val ((m <Perception2D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:arg-val is deprecated.  Use tabletop_2D_segmentation-srv:arg instead.")
  (arg m))

(cl:ensure-generic-function 'arg2-val :lambda-list '(m))
(cl:defmethod arg2-val ((m <Perception2D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:arg2-val is deprecated.  Use tabletop_2D_segmentation-srv:arg2 instead.")
  (arg2 m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Perception2D-request>)))
    "Constants for message type '<Perception2D-request>"
  '((:DO_PERCEPT . 0)
    (:EXTRACT_EFFECT . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Perception2D-request)))
    "Constants for message type 'Perception2D-request"
  '((:DO_PERCEPT . 0)
    (:EXTRACT_EFFECT . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Perception2D-request>) ostream)
  "Serializes a message object of type '<Perception2D-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'arg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'arg2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Perception2D-request>) istream)
  "Deserializes a message object of type '<Perception2D-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arg) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arg2) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Perception2D-request>)))
  "Returns string type for a service object of type '<Perception2D-request>"
  "tabletop_2D_segmentation/Perception2DRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Perception2D-request)))
  "Returns string type for a service object of type 'Perception2D-request"
  "tabletop_2D_segmentation/Perception2DRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Perception2D-request>)))
  "Returns md5sum for a message object of type '<Perception2D-request>"
  "1da76ff735fd425703f5cdeb93da420c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Perception2D-request)))
  "Returns md5sum for a message object of type 'Perception2D-request"
  "1da76ff735fd425703f5cdeb93da420c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Perception2D-request>)))
  "Returns full string definition for message of type '<Perception2D-request>"
  (cl:format cl:nil "uint8 DO_PERCEPT = 0~%uint8 EXTRACT_EFFECT = 1~%uint8 task~%int8 arg~%int8 arg2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Perception2D-request)))
  "Returns full string definition for message of type 'Perception2D-request"
  (cl:format cl:nil "uint8 DO_PERCEPT = 0~%uint8 EXTRACT_EFFECT = 1~%uint8 task~%int8 arg~%int8 arg2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Perception2D-request>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Perception2D-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Perception2D-request
    (cl:cons ':task (task msg))
    (cl:cons ':arg (arg msg))
    (cl:cons ':arg2 (arg2 msg))
))
;//! \htmlinclude Perception2D-response.msg.html

(cl:defclass <Perception2D-response> (roslisp-msg-protocol:ros-message)
  ((raw_image
    :reader raw_image
    :initarg :raw_image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (filtered_image
    :reader filtered_image
    :initarg :filtered_image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (roi
    :reader roi
    :initarg :roi
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest))
   (face_detected
    :reader face_detected
    :initarg :face_detected
    :type cl:boolean
    :initform cl:nil)
   (ooi_area
    :reader ooi_area
    :initarg :ooi_area
    :type cl:fixnum
    :initform 0)
   (ooi_color_r_hist
    :reader ooi_color_r_hist
    :initarg :ooi_color_r_hist
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (ooi_color_g_hist
    :reader ooi_color_g_hist
    :initarg :ooi_color_g_hist
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (ooi_color_b_hist
    :reader ooi_color_b_hist
    :initarg :ooi_color_b_hist
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Perception2D-response (<Perception2D-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Perception2D-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Perception2D-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tabletop_2D_segmentation-srv:<Perception2D-response> is deprecated: use tabletop_2D_segmentation-srv:Perception2D-response instead.")))

(cl:ensure-generic-function 'raw_image-val :lambda-list '(m))
(cl:defmethod raw_image-val ((m <Perception2D-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:raw_image-val is deprecated.  Use tabletop_2D_segmentation-srv:raw_image instead.")
  (raw_image m))

(cl:ensure-generic-function 'filtered_image-val :lambda-list '(m))
(cl:defmethod filtered_image-val ((m <Perception2D-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:filtered_image-val is deprecated.  Use tabletop_2D_segmentation-srv:filtered_image instead.")
  (filtered_image m))

(cl:ensure-generic-function 'roi-val :lambda-list '(m))
(cl:defmethod roi-val ((m <Perception2D-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:roi-val is deprecated.  Use tabletop_2D_segmentation-srv:roi instead.")
  (roi m))

(cl:ensure-generic-function 'face_detected-val :lambda-list '(m))
(cl:defmethod face_detected-val ((m <Perception2D-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:face_detected-val is deprecated.  Use tabletop_2D_segmentation-srv:face_detected instead.")
  (face_detected m))

(cl:ensure-generic-function 'ooi_area-val :lambda-list '(m))
(cl:defmethod ooi_area-val ((m <Perception2D-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:ooi_area-val is deprecated.  Use tabletop_2D_segmentation-srv:ooi_area instead.")
  (ooi_area m))

(cl:ensure-generic-function 'ooi_color_r_hist-val :lambda-list '(m))
(cl:defmethod ooi_color_r_hist-val ((m <Perception2D-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:ooi_color_r_hist-val is deprecated.  Use tabletop_2D_segmentation-srv:ooi_color_r_hist instead.")
  (ooi_color_r_hist m))

(cl:ensure-generic-function 'ooi_color_g_hist-val :lambda-list '(m))
(cl:defmethod ooi_color_g_hist-val ((m <Perception2D-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:ooi_color_g_hist-val is deprecated.  Use tabletop_2D_segmentation-srv:ooi_color_g_hist instead.")
  (ooi_color_g_hist m))

(cl:ensure-generic-function 'ooi_color_b_hist-val :lambda-list '(m))
(cl:defmethod ooi_color_b_hist-val ((m <Perception2D-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tabletop_2D_segmentation-srv:ooi_color_b_hist-val is deprecated.  Use tabletop_2D_segmentation-srv:ooi_color_b_hist instead.")
  (ooi_color_b_hist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Perception2D-response>) ostream)
  "Serializes a message object of type '<Perception2D-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'raw_image) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'filtered_image) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'roi) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'face_detected) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'ooi_area)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ooi_color_r_hist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'ooi_color_r_hist))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ooi_color_g_hist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'ooi_color_g_hist))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ooi_color_b_hist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'ooi_color_b_hist))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Perception2D-response>) istream)
  "Deserializes a message object of type '<Perception2D-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'raw_image) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'filtered_image) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'roi) istream)
    (cl:setf (cl:slot-value msg 'face_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ooi_area) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ooi_color_r_hist) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ooi_color_r_hist)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ooi_color_g_hist) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ooi_color_g_hist)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ooi_color_b_hist) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ooi_color_b_hist)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Perception2D-response>)))
  "Returns string type for a service object of type '<Perception2D-response>"
  "tabletop_2D_segmentation/Perception2DResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Perception2D-response)))
  "Returns string type for a service object of type 'Perception2D-response"
  "tabletop_2D_segmentation/Perception2DResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Perception2D-response>)))
  "Returns md5sum for a message object of type '<Perception2D-response>"
  "1da76ff735fd425703f5cdeb93da420c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Perception2D-response)))
  "Returns md5sum for a message object of type 'Perception2D-response"
  "1da76ff735fd425703f5cdeb93da420c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Perception2D-response>)))
  "Returns full string definition for message of type '<Perception2D-response>"
  (cl:format cl:nil "~%~%~%sensor_msgs/Image raw_image~%sensor_msgs/Image filtered_image~%sensor_msgs/RegionOfInterest roi~%bool face_detected~%int16 ooi_area~%int8[] ooi_color_r_hist~%int8[] ooi_color_g_hist~%int8[] ooi_color_b_hist~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Perception2D-response)))
  "Returns full string definition for message of type 'Perception2D-response"
  (cl:format cl:nil "~%~%~%sensor_msgs/Image raw_image~%sensor_msgs/Image filtered_image~%sensor_msgs/RegionOfInterest roi~%bool face_detected~%int16 ooi_area~%int8[] ooi_color_r_hist~%int8[] ooi_color_g_hist~%int8[] ooi_color_b_hist~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Perception2D-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'raw_image))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'filtered_image))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'roi))
     1
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ooi_color_r_hist) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ooi_color_g_hist) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ooi_color_b_hist) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Perception2D-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Perception2D-response
    (cl:cons ':raw_image (raw_image msg))
    (cl:cons ':filtered_image (filtered_image msg))
    (cl:cons ':roi (roi msg))
    (cl:cons ':face_detected (face_detected msg))
    (cl:cons ':ooi_area (ooi_area msg))
    (cl:cons ':ooi_color_r_hist (ooi_color_r_hist msg))
    (cl:cons ':ooi_color_g_hist (ooi_color_g_hist msg))
    (cl:cons ':ooi_color_b_hist (ooi_color_b_hist msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Perception2D)))
  'Perception2D-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Perception2D)))
  'Perception2D-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Perception2D)))
  "Returns string type for a service object of type '<Perception2D>"
  "tabletop_2D_segmentation/Perception2D")