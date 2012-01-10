; Auto-generated. Do not edit!


(cl:in-package aff_msgs-msg)


;//! \htmlinclude Speech.msg.html

(cl:defclass <Speech> (roslisp-msg-protocol:ros-message)
  ((speech_cmd
    :reader speech_cmd
    :initarg :speech_cmd
    :type cl:fixnum
    :initform 0)
   (speech_arg
    :reader speech_arg
    :initarg :speech_arg
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Speech (<Speech>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Speech>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Speech)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aff_msgs-msg:<Speech> is deprecated: use aff_msgs-msg:Speech instead.")))

(cl:ensure-generic-function 'speech_cmd-val :lambda-list '(m))
(cl:defmethod speech_cmd-val ((m <Speech>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:speech_cmd-val is deprecated.  Use aff_msgs-msg:speech_cmd instead.")
  (speech_cmd m))

(cl:ensure-generic-function 'speech_arg-val :lambda-list '(m))
(cl:defmethod speech_arg-val ((m <Speech>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:speech_arg-val is deprecated.  Use aff_msgs-msg:speech_arg instead.")
  (speech_arg m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Speech>)))
    "Constants for message type '<Speech>"
  '((:MAX_ACTION_INDEX . 19)
    (:MAX_EFFECT_INDEX . 39)
    (:PUSH_RIGHT . 0)
    (:PUSH_LEFT . 1)
    (:PUSH_FORWARD . 2)
    (:PUSH_BACKWARD . 3)
    (:LIFT . 4)
    (:GRASP . 5)
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
    (:PUSHED_RIGHT . 20)
    (:PUSHED_LEFT . 21)
    (:PUSHED_FORWARD . 22)
    (:PUSHED_BACKWARD . 23)
    (:ROLLED_RIGHT . 24)
    (:ROLLED_LEFT . 25)
    (:ROLLED_FORWARD . 26)
    (:LIFTED . 27)
    (:DISAPPEARED . 28)
    (:NO_EFFECT . 29)
    (:GRASPED . 30)
    (:DONTCARE . 31)
    (:NO_CHANGE . 32)
    (:REACHED . 33)
    (:ACQUIRED . 34)
    (:RELEASED . 35)
    (:TAKEN . 36)
    (:GIVEN . 37)
    (:CONTINUE . 38))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Speech)))
    "Constants for message type 'Speech"
  '((:MAX_ACTION_INDEX . 19)
    (:MAX_EFFECT_INDEX . 39)
    (:PUSH_RIGHT . 0)
    (:PUSH_LEFT . 1)
    (:PUSH_FORWARD . 2)
    (:PUSH_BACKWARD . 3)
    (:LIFT . 4)
    (:GRASP . 5)
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
    (:PUSHED_RIGHT . 20)
    (:PUSHED_LEFT . 21)
    (:PUSHED_FORWARD . 22)
    (:PUSHED_BACKWARD . 23)
    (:ROLLED_RIGHT . 24)
    (:ROLLED_LEFT . 25)
    (:ROLLED_FORWARD . 26)
    (:LIFTED . 27)
    (:DISAPPEARED . 28)
    (:NO_EFFECT . 29)
    (:GRASPED . 30)
    (:DONTCARE . 31)
    (:NO_CHANGE . 32)
    (:REACHED . 33)
    (:ACQUIRED . 34)
    (:RELEASED . 35)
    (:TAKEN . 36)
    (:GIVEN . 37)
    (:CONTINUE . 38))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Speech>) ostream)
  "Serializes a message object of type '<Speech>"
  (cl:let* ((signed (cl:slot-value msg 'speech_cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speech_arg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Speech>) istream)
  "Deserializes a message object of type '<Speech>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speech_cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speech_arg) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Speech>)))
  "Returns string type for a message object of type '<Speech>"
  "aff_msgs/Speech")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Speech)))
  "Returns string type for a message object of type 'Speech"
  "aff_msgs/Speech")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Speech>)))
  "Returns md5sum for a message object of type '<Speech>"
  "599975281b07156156eaaae710a35947")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Speech)))
  "Returns md5sum for a message object of type 'Speech"
  "599975281b07156156eaaae710a35947")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Speech>)))
  "Returns full string definition for message of type '<Speech>"
  (cl:format cl:nil "int8 MAX_ACTION_INDEX = 19~%int8 MAX_EFFECT_INDEX = 39~%int8 PUSH_RIGHT = 0~%int8 PUSH_LEFT =1~%int8 PUSH_FORWARD =2~%int8 PUSH_BACKWARD=3~%int8 LIFT=4~%int8 GRASP=5~%int8 HOME=6~%int8 POINT=7~%int8 HIDE=8~%int8 CANCEL=9~%int8 STOP=10~%int8 SHOW=11~%int8 TUCK_ARMS=12~%int8 LOOK_AT_REGION=13~%int8 LOOK_AT_POINT=14~%int8 LOOK_AT_FACE=15~%int8 REACH=16~%int8 TAKE=17~%int8 GIVE=18~%int8 RELEASE=19~%int8 PUSHED_RIGHT=20~%int8 PUSHED_LEFT=21~%int8 PUSHED_FORWARD=22~%int8 PUSHED_BACKWARD=23~%int8 ROLLED_RIGHT=24~%int8 ROLLED_LEFT=25~%int8 ROLLED_FORWARD=26~%int8 LIFTED=27~%int8 DISAPPEARED=28~%int8 NO_EFFECT=29~%int8 GRASPED=30~%int8 DONTCARE=31~%int8 NO_CHANGE=32~%int8 REACHED=33~%int8 ACQUIRED=34~%int8 RELEASED=35~%int8 TAKEN=36~%int8 GIVEN=37~%int8 CONTINUE=38~%~%int8 speech_cmd~%int8 speech_arg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Speech)))
  "Returns full string definition for message of type 'Speech"
  (cl:format cl:nil "int8 MAX_ACTION_INDEX = 19~%int8 MAX_EFFECT_INDEX = 39~%int8 PUSH_RIGHT = 0~%int8 PUSH_LEFT =1~%int8 PUSH_FORWARD =2~%int8 PUSH_BACKWARD=3~%int8 LIFT=4~%int8 GRASP=5~%int8 HOME=6~%int8 POINT=7~%int8 HIDE=8~%int8 CANCEL=9~%int8 STOP=10~%int8 SHOW=11~%int8 TUCK_ARMS=12~%int8 LOOK_AT_REGION=13~%int8 LOOK_AT_POINT=14~%int8 LOOK_AT_FACE=15~%int8 REACH=16~%int8 TAKE=17~%int8 GIVE=18~%int8 RELEASE=19~%int8 PUSHED_RIGHT=20~%int8 PUSHED_LEFT=21~%int8 PUSHED_FORWARD=22~%int8 PUSHED_BACKWARD=23~%int8 ROLLED_RIGHT=24~%int8 ROLLED_LEFT=25~%int8 ROLLED_FORWARD=26~%int8 LIFTED=27~%int8 DISAPPEARED=28~%int8 NO_EFFECT=29~%int8 GRASPED=30~%int8 DONTCARE=31~%int8 NO_CHANGE=32~%int8 REACHED=33~%int8 ACQUIRED=34~%int8 RELEASED=35~%int8 TAKEN=36~%int8 GIVEN=37~%int8 CONTINUE=38~%~%int8 speech_cmd~%int8 speech_arg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Speech>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Speech>))
  "Converts a ROS message object to a list"
  (cl:list 'Speech
    (cl:cons ':speech_cmd (speech_cmd msg))
    (cl:cons ':speech_arg (speech_arg msg))
))
