; Auto-generated. Do not edit!


(cl:in-package aff_msgs-msg)


;//! \htmlinclude Features.msg.html

(cl:defclass <Features> (roslisp-msg-protocol:ros-message)
  ((feature_type
    :reader feature_type
    :initarg :feature_type
    :type cl:fixnum
    :initform 0)
   (feature_class
    :reader feature_class
    :initarg :feature_class
    :type cl:fixnum
    :initform 0)
   (features
    :reader features
    :initarg :features
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (episode_index
    :reader episode_index
    :initarg :episode_index
    :type cl:integer
    :initform 0))
)

(cl:defclass Features (<Features>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Features>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Features)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aff_msgs-msg:<Features> is deprecated: use aff_msgs-msg:Features instead.")))

(cl:ensure-generic-function 'feature_type-val :lambda-list '(m))
(cl:defmethod feature_type-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:feature_type-val is deprecated.  Use aff_msgs-msg:feature_type instead.")
  (feature_type m))

(cl:ensure-generic-function 'feature_class-val :lambda-list '(m))
(cl:defmethod feature_class-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:feature_class-val is deprecated.  Use aff_msgs-msg:feature_class instead.")
  (feature_class m))

(cl:ensure-generic-function 'features-val :lambda-list '(m))
(cl:defmethod features-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:features-val is deprecated.  Use aff_msgs-msg:features instead.")
  (features m))

(cl:ensure-generic-function 'episode_index-val :lambda-list '(m))
(cl:defmethod episode_index-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:episode_index-val is deprecated.  Use aff_msgs-msg:episode_index instead.")
  (episode_index m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Features>)))
    "Constants for message type '<Features>"
  '((:ENTITY . 0)
    (:EFFECT . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Features)))
    "Constants for message type 'Features"
  '((:ENTITY . 0)
    (:EFFECT . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Features>) ostream)
  "Serializes a message object of type '<Features>"
  (cl:let* ((signed (cl:slot-value msg 'feature_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'feature_class)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'features))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'features))
  (cl:let* ((signed (cl:slot-value msg 'episode_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Features>) istream)
  "Deserializes a message object of type '<Features>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'feature_type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'feature_class) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'features) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'features)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'episode_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Features>)))
  "Returns string type for a message object of type '<Features>"
  "aff_msgs/Features")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Features)))
  "Returns string type for a message object of type 'Features"
  "aff_msgs/Features")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Features>)))
  "Returns md5sum for a message object of type '<Features>"
  "54b87ae43f21551d16ac92ead68b044d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Features)))
  "Returns md5sum for a message object of type 'Features"
  "54b87ae43f21551d16ac92ead68b044d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Features>)))
  "Returns full string definition for message of type '<Features>"
  (cl:format cl:nil "int8 ENTITY=0~%int8 EFFECT=1~%int8 feature_type~%int8 feature_class #insert supervised feature class if applicable~%float32[] features~%int32 episode_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Features)))
  "Returns full string definition for message of type 'Features"
  (cl:format cl:nil "int8 ENTITY=0~%int8 EFFECT=1~%int8 feature_type~%int8 feature_class #insert supervised feature class if applicable~%float32[] features~%int32 episode_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Features>))
  (cl:+ 0
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'features) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Features>))
  "Converts a ROS message object to a list"
  (cl:list 'Features
    (cl:cons ':feature_type (feature_type msg))
    (cl:cons ':feature_class (feature_class msg))
    (cl:cons ':features (features msg))
    (cl:cons ':episode_index (episode_index msg))
))
