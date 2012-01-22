; Auto-generated. Do not edit!


(cl:in-package behavior_manager-srv)


;//! \htmlinclude Action-request.msg.html

(cl:defclass <Action-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Action-request (<Action-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Action-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Action-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_manager-srv:<Action-request> is deprecated: use behavior_manager-srv:Action-request instead.")))

(cl:ensure-generic-function 'task-val :lambda-list '(m))
(cl:defmethod task-val ((m <Action-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_manager-srv:task-val is deprecated.  Use behavior_manager-srv:task instead.")
  (task m))

(cl:ensure-generic-function 'arg-val :lambda-list '(m))
(cl:defmethod arg-val ((m <Action-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_manager-srv:arg-val is deprecated.  Use behavior_manager-srv:arg instead.")
  (arg m))

(cl:ensure-generic-function 'pushable_object_center-val :lambda-list '(m))
(cl:defmethod pushable_object_center-val ((m <Action-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_manager-srv:pushable_object_center-val is deprecated.  Use behavior_manager-srv:pushable_object_center instead.")
  (pushable_object_center m))

(cl:ensure-generic-function 'pushable_object_size-val :lambda-list '(m))
(cl:defmethod pushable_object_size-val ((m <Action-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_manager-srv:pushable_object_size-val is deprecated.  Use behavior_manager-srv:pushable_object_size instead.")
  (pushable_object_size m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Action-request>)))
    "Constants for message type '<Action-request>"
  '((:DONT_ACT . -1)
    (:PUSH_RIGHT . 0)
    (:PUSH_LEFT . 1)
    (:PUSH_RIGHT_UPPER . 24)
    (:PUSH_LEFT_UPPER . 25)
    (:PUSH_FORWARD . 2)
    (:PUSH_BACKWARD . 3)
    (:LIFT . 4)
    (:GRASP . 5)
    (:GRASP_UPPER . 26)
    (:HOME . 6)
    (:POINT . 7)
    (:HIDE . 8)
    (:CANCEL . 9)
    (:STOP . 10)
    (:SHOW . 11)
    (:TUCK_ARMS . 12)
    (:LOOK_AT_REGION . 13)
    (:LOOK_AT_POINT . 14)
    (:LOOK_AT_FACE . 15)
    (:REACH . 16)
    (:TAKE . 17)
    (:GIVE . 18)
    (:RELEASE . 19)
    (:RELEASE_UPWARD . 20)
    (:RELEASE_DOWNWARD . 21)
    (:COVER . 22)
    (:DROP . 23)
    (:DETECT_TOUCH . 27))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Action-request)))
    "Constants for message type 'Action-request"
  '((:DONT_ACT . -1)
    (:PUSH_RIGHT . 0)
    (:PUSH_LEFT . 1)
    (:PUSH_RIGHT_UPPER . 24)
    (:PUSH_LEFT_UPPER . 25)
    (:PUSH_FORWARD . 2)
    (:PUSH_BACKWARD . 3)
    (:LIFT . 4)
    (:GRASP . 5)
    (:GRASP_UPPER . 26)
    (:HOME . 6)
    (:POINT . 7)
    (:HIDE . 8)
    (:CANCEL . 9)
    (:STOP . 10)
    (:SHOW . 11)
    (:TUCK_ARMS . 12)
    (:LOOK_AT_REGION . 13)
    (:LOOK_AT_POINT . 14)
    (:LOOK_AT_FACE . 15)
    (:REACH . 16)
    (:TAKE . 17)
    (:GIVE . 18)
    (:RELEASE . 19)
    (:RELEASE_UPWARD . 20)
    (:RELEASE_DOWNWARD . 21)
    (:COVER . 22)
    (:DROP . 23)
    (:DETECT_TOUCH . 27))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Action-request>) ostream)
  "Serializes a message object of type '<Action-request>"
  (cl:let* ((signed (cl:slot-value msg 'task)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'arg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Action-request>) istream)
  "Deserializes a message object of type '<Action-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arg) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Action-request>)))
  "Returns string type for a service object of type '<Action-request>"
  "behavior_manager/ActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Action-request)))
  "Returns string type for a service object of type 'Action-request"
  "behavior_manager/ActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Action-request>)))
  "Returns md5sum for a message object of type '<Action-request>"
  "15273ca2a759dcd8367407dc29b863d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Action-request)))
  "Returns md5sum for a message object of type 'Action-request"
  "15273ca2a759dcd8367407dc29b863d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Action-request>)))
  "Returns full string definition for message of type '<Action-request>"
  (cl:format cl:nil "int8 DONT_ACT = -1~%int8 PUSH_RIGHT = 0~%int8 PUSH_LEFT =1~%int8 PUSH_RIGHT_UPPER = 24~%int8 PUSH_LEFT_UPPER = 25~%int8 PUSH_FORWARD =2~%int8 PUSH_BACKWARD=3~%int8 LIFT=4~%int8 GRASP=5~%int8 GRASP_UPPER=26~%int8 HOME=6~%int8 POINT=7~%int8 HIDE=8~%int8 CANCEL=9~%int8 STOP=10~%int8 SHOW=11~%int8 TUCK_ARMS=12~%int8 LOOK_AT_REGION=13~%int8 LOOK_AT_POINT=14~%int8 LOOK_AT_FACE=15~%int8 REACH=16~%int8 TAKE=17~%int8 GIVE=18~%int8 RELEASE=19~%int8 RELEASE_UPWARD=20~%int8 RELEASE_DOWNWARD=21~%int8 COVER=22~%int8 DROP=23~%int8 DETECT_TOUCH=27~%int8 task~%int8 arg~%float32[] pushable_object_center~%float32[] pushable_object_size~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Action-request)))
  "Returns full string definition for message of type 'Action-request"
  (cl:format cl:nil "int8 DONT_ACT = -1~%int8 PUSH_RIGHT = 0~%int8 PUSH_LEFT =1~%int8 PUSH_RIGHT_UPPER = 24~%int8 PUSH_LEFT_UPPER = 25~%int8 PUSH_FORWARD =2~%int8 PUSH_BACKWARD=3~%int8 LIFT=4~%int8 GRASP=5~%int8 GRASP_UPPER=26~%int8 HOME=6~%int8 POINT=7~%int8 HIDE=8~%int8 CANCEL=9~%int8 STOP=10~%int8 SHOW=11~%int8 TUCK_ARMS=12~%int8 LOOK_AT_REGION=13~%int8 LOOK_AT_POINT=14~%int8 LOOK_AT_FACE=15~%int8 REACH=16~%int8 TAKE=17~%int8 GIVE=18~%int8 RELEASE=19~%int8 RELEASE_UPWARD=20~%int8 RELEASE_DOWNWARD=21~%int8 COVER=22~%int8 DROP=23~%int8 DETECT_TOUCH=27~%int8 task~%int8 arg~%float32[] pushable_object_center~%float32[] pushable_object_size~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Action-request>))
  (cl:+ 0
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pushable_object_center) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pushable_object_size) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Action-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Action-request
    (cl:cons ':task (task msg))
    (cl:cons ':arg (arg msg))
    (cl:cons ':pushable_object_center (pushable_object_center msg))
    (cl:cons ':pushable_object_size (pushable_object_size msg))
))
;//! \htmlinclude Action-response.msg.html

(cl:defclass <Action-response> (roslisp-msg-protocol:ros-message)
  ((feedback
    :reader feedback
    :initarg :feedback
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Action-response (<Action-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Action-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Action-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_manager-srv:<Action-response> is deprecated: use behavior_manager-srv:Action-response instead.")))

(cl:ensure-generic-function 'feedback-val :lambda-list '(m))
(cl:defmethod feedback-val ((m <Action-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_manager-srv:feedback-val is deprecated.  Use behavior_manager-srv:feedback instead.")
  (feedback m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Action-response>)))
    "Constants for message type '<Action-response>"
  '((:DONE . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Action-response)))
    "Constants for message type 'Action-response"
  '((:DONE . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Action-response>) ostream)
  "Serializes a message object of type '<Action-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedback)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Action-response>) istream)
  "Deserializes a message object of type '<Action-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedback)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Action-response>)))
  "Returns string type for a service object of type '<Action-response>"
  "behavior_manager/ActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Action-response)))
  "Returns string type for a service object of type 'Action-response"
  "behavior_manager/ActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Action-response>)))
  "Returns md5sum for a message object of type '<Action-response>"
  "15273ca2a759dcd8367407dc29b863d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Action-response)))
  "Returns md5sum for a message object of type 'Action-response"
  "15273ca2a759dcd8367407dc29b863d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Action-response>)))
  "Returns full string definition for message of type '<Action-response>"
  (cl:format cl:nil "~%uint8 DONE = 1~%uint8 feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Action-response)))
  "Returns full string definition for message of type 'Action-response"
  (cl:format cl:nil "~%uint8 DONE = 1~%uint8 feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Action-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Action-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Action-response
    (cl:cons ':feedback (feedback msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Action)))
  'Action-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Action)))
  'Action-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Action)))
  "Returns string type for a service object of type '<Action>"
  "behavior_manager/Action")