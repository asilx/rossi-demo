; Auto-generated. Do not edit!


(cl:in-package aff_msgs-msg)


;//! \htmlinclude ObjectOfInterest.msg.html

(cl:defclass <ObjectOfInterest> (roslisp-msg-protocol:ros-message)
  ((object_center
    :reader object_center
    :initarg :object_center
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (object_size
    :reader object_size
    :initarg :object_size
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (affordances
    :reader affordances
    :initarg :affordances
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ObjectOfInterest (<ObjectOfInterest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectOfInterest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectOfInterest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aff_msgs-msg:<ObjectOfInterest> is deprecated: use aff_msgs-msg:ObjectOfInterest instead.")))

(cl:ensure-generic-function 'object_center-val :lambda-list '(m))
(cl:defmethod object_center-val ((m <ObjectOfInterest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:object_center-val is deprecated.  Use aff_msgs-msg:object_center instead.")
  (object_center m))

(cl:ensure-generic-function 'object_size-val :lambda-list '(m))
(cl:defmethod object_size-val ((m <ObjectOfInterest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:object_size-val is deprecated.  Use aff_msgs-msg:object_size instead.")
  (object_size m))

(cl:ensure-generic-function 'affordances-val :lambda-list '(m))
(cl:defmethod affordances-val ((m <ObjectOfInterest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:affordances-val is deprecated.  Use aff_msgs-msg:affordances instead.")
  (affordances m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectOfInterest>) ostream)
  "Serializes a message object of type '<ObjectOfInterest>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'object_center))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'object_center))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'object_size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'object_size))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'affordances))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'affordances))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectOfInterest>) istream)
  "Deserializes a message object of type '<ObjectOfInterest>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'object_center) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'object_center)))
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
  (cl:setf (cl:slot-value msg 'object_size) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'object_size)))
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
  (cl:setf (cl:slot-value msg 'affordances) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'affordances)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectOfInterest>)))
  "Returns string type for a message object of type '<ObjectOfInterest>"
  "aff_msgs/ObjectOfInterest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectOfInterest)))
  "Returns string type for a message object of type 'ObjectOfInterest"
  "aff_msgs/ObjectOfInterest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectOfInterest>)))
  "Returns md5sum for a message object of type '<ObjectOfInterest>"
  "a29efac146671dfe2b7bc551235b000c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectOfInterest)))
  "Returns md5sum for a message object of type 'ObjectOfInterest"
  "a29efac146671dfe2b7bc551235b000c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectOfInterest>)))
  "Returns full string definition for message of type '<ObjectOfInterest>"
  (cl:format cl:nil "float32[] object_center~%float32[] object_size~%string[] affordances~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectOfInterest)))
  "Returns full string definition for message of type 'ObjectOfInterest"
  (cl:format cl:nil "float32[] object_center~%float32[] object_size~%string[] affordances~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectOfInterest>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object_center) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object_size) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'affordances) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectOfInterest>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectOfInterest
    (cl:cons ':object_center (object_center msg))
    (cl:cons ':object_size (object_size msg))
    (cl:cons ':affordances (affordances msg))
))
