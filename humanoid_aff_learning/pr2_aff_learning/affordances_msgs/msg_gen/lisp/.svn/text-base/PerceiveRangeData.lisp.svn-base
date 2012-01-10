; Auto-generated. Do not edit!


(cl:in-package affordances_msgs-msg)


;//! \htmlinclude PerceiveRangeData.msg.html

(cl:defclass <PerceiveRangeData> (roslisp-msg-protocol:ros-message)
  ((do_perceive
    :reader do_perceive
    :initarg :do_perceive
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PerceiveRangeData (<PerceiveRangeData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PerceiveRangeData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PerceiveRangeData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name affordances_msgs-msg:<PerceiveRangeData> is deprecated: use affordances_msgs-msg:PerceiveRangeData instead.")))

(cl:ensure-generic-function 'do_perceive-val :lambda-list '(m))
(cl:defmethod do_perceive-val ((m <PerceiveRangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader affordances_msgs-msg:do_perceive-val is deprecated.  Use affordances_msgs-msg:do_perceive instead.")
  (do_perceive m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PerceiveRangeData>) ostream)
  "Serializes a message object of type '<PerceiveRangeData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'do_perceive) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PerceiveRangeData>) istream)
  "Deserializes a message object of type '<PerceiveRangeData>"
    (cl:setf (cl:slot-value msg 'do_perceive) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PerceiveRangeData>)))
  "Returns string type for a message object of type '<PerceiveRangeData>"
  "affordances_msgs/PerceiveRangeData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerceiveRangeData)))
  "Returns string type for a message object of type 'PerceiveRangeData"
  "affordances_msgs/PerceiveRangeData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PerceiveRangeData>)))
  "Returns md5sum for a message object of type '<PerceiveRangeData>"
  "8addac4e4f12ab38392d4ae360422b58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PerceiveRangeData)))
  "Returns md5sum for a message object of type 'PerceiveRangeData"
  "8addac4e4f12ab38392d4ae360422b58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PerceiveRangeData>)))
  "Returns full string definition for message of type '<PerceiveRangeData>"
  (cl:format cl:nil "bool do_perceive~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PerceiveRangeData)))
  "Returns full string definition for message of type 'PerceiveRangeData"
  (cl:format cl:nil "bool do_perceive~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PerceiveRangeData>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PerceiveRangeData>))
  "Converts a ROS message object to a list"
  (cl:list 'PerceiveRangeData
    (cl:cons ':do_perceive (do_perceive msg))
))
