; Auto-generated. Do not edit!


(cl:in-package feature_manager-srv)


;//! \htmlinclude Perception-request.msg.html

(cl:defclass <Perception-request> (roslisp-msg-protocol:ros-message)
  ((task
    :reader task
    :initarg :task
    :type cl:fixnum
    :initform 0)
   (arg
    :reader arg
    :initarg :arg
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Perception-request (<Perception-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Perception-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Perception-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feature_manager-srv:<Perception-request> is deprecated: use feature_manager-srv:Perception-request instead.")))

(cl:ensure-generic-function 'task-val :lambda-list '(m))
(cl:defmethod task-val ((m <Perception-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_manager-srv:task-val is deprecated.  Use feature_manager-srv:task instead.")
  (task m))

(cl:ensure-generic-function 'arg-val :lambda-list '(m))
(cl:defmethod arg-val ((m <Perception-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_manager-srv:arg-val is deprecated.  Use feature_manager-srv:arg instead.")
  (arg m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Perception-request>)))
    "Constants for message type '<Perception-request>"
  '((:DO_PERCEPT . 0)
    (:EXTRACT_EFFECT . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Perception-request)))
    "Constants for message type 'Perception-request"
  '((:DO_PERCEPT . 0)
    (:EXTRACT_EFFECT . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Perception-request>) ostream)
  "Serializes a message object of type '<Perception-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'arg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Perception-request>) istream)
  "Deserializes a message object of type '<Perception-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arg) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Perception-request>)))
  "Returns string type for a service object of type '<Perception-request>"
  "feature_manager/PerceptionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Perception-request)))
  "Returns string type for a service object of type 'Perception-request"
  "feature_manager/PerceptionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Perception-request>)))
  "Returns md5sum for a message object of type '<Perception-request>"
  "546737ade87225c9c103435e892ab5cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Perception-request)))
  "Returns md5sum for a message object of type 'Perception-request"
  "546737ade87225c9c103435e892ab5cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Perception-request>)))
  "Returns full string definition for message of type '<Perception-request>"
  (cl:format cl:nil "uint8 DO_PERCEPT = 0~%uint8 EXTRACT_EFFECT = 1~%uint8 task~%int8 arg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Perception-request)))
  "Returns full string definition for message of type 'Perception-request"
  (cl:format cl:nil "uint8 DO_PERCEPT = 0~%uint8 EXTRACT_EFFECT = 1~%uint8 task~%int8 arg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Perception-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Perception-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Perception-request
    (cl:cons ':task (task msg))
    (cl:cons ':arg (arg msg))
))
;//! \htmlinclude Perception-response.msg.html

(cl:defclass <Perception-response> (roslisp-msg-protocol:ros-message)
  ((feedback
    :reader feedback
    :initarg :feedback
    :type cl:fixnum
    :initform 0)
   (pushable_object_center
    :reader pushable_object_center
    :initarg :pushable_object_center
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (pushable_object_size
    :reader pushable_object_size
    :initarg :pushable_object_size
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Perception-response (<Perception-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Perception-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Perception-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feature_manager-srv:<Perception-response> is deprecated: use feature_manager-srv:Perception-response instead.")))

(cl:ensure-generic-function 'feedback-val :lambda-list '(m))
(cl:defmethod feedback-val ((m <Perception-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_manager-srv:feedback-val is deprecated.  Use feature_manager-srv:feedback instead.")
  (feedback m))

(cl:ensure-generic-function 'pushable_object_center-val :lambda-list '(m))
(cl:defmethod pushable_object_center-val ((m <Perception-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_manager-srv:pushable_object_center-val is deprecated.  Use feature_manager-srv:pushable_object_center instead.")
  (pushable_object_center m))

(cl:ensure-generic-function 'pushable_object_size-val :lambda-list '(m))
(cl:defmethod pushable_object_size-val ((m <Perception-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feature_manager-srv:pushable_object_size-val is deprecated.  Use feature_manager-srv:pushable_object_size instead.")
  (pushable_object_size m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Perception-response>)))
    "Constants for message type '<Perception-response>"
  '((:DONE . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Perception-response)))
    "Constants for message type 'Perception-response"
  '((:DONE . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Perception-response>) ostream)
  "Serializes a message object of type '<Perception-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedback)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pushable_object_center))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pushable_object_center))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pushable_object_size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pushable_object_size))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Perception-response>) istream)
  "Deserializes a message object of type '<Perception-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedback)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pushable_object_center) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pushable_object_center)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pushable_object_size) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pushable_object_size)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Perception-response>)))
  "Returns string type for a service object of type '<Perception-response>"
  "feature_manager/PerceptionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Perception-response)))
  "Returns string type for a service object of type 'Perception-response"
  "feature_manager/PerceptionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Perception-response>)))
  "Returns md5sum for a message object of type '<Perception-response>"
  "546737ade87225c9c103435e892ab5cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Perception-response)))
  "Returns md5sum for a message object of type 'Perception-response"
  "546737ade87225c9c103435e892ab5cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Perception-response>)))
  "Returns full string definition for message of type '<Perception-response>"
  (cl:format cl:nil "~%uint8 DONE = 1~%uint8 feedback~%~%float32[] pushable_object_center~%float32[] pushable_object_size~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Perception-response)))
  "Returns full string definition for message of type 'Perception-response"
  (cl:format cl:nil "~%uint8 DONE = 1~%uint8 feedback~%~%float32[] pushable_object_center~%float32[] pushable_object_size~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Perception-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pushable_object_center) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pushable_object_size) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Perception-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Perception-response
    (cl:cons ':feedback (feedback msg))
    (cl:cons ':pushable_object_center (pushable_object_center msg))
    (cl:cons ':pushable_object_size (pushable_object_size msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Perception)))
  'Perception-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Perception)))
  'Perception-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Perception)))
  "Returns string type for a service object of type '<Perception>"
  "feature_manager/Perception")