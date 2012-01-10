; Auto-generated. Do not edit!


(cl:in-package pr2_learning_affordances-srv)


;//! \htmlinclude ExecuteCartesianIKTrajectory-request.msg.html

(cl:defclass <ExecuteCartesianIKTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass ExecuteCartesianIKTrajectory-request (<ExecuteCartesianIKTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteCartesianIKTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteCartesianIKTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pr2_learning_affordances-srv:<ExecuteCartesianIKTrajectory-request> is deprecated: use pr2_learning_affordances-srv:ExecuteCartesianIKTrajectory-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ExecuteCartesianIKTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pr2_learning_affordances-srv:header-val is deprecated.  Use pr2_learning_affordances-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <ExecuteCartesianIKTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pr2_learning_affordances-srv:poses-val is deprecated.  Use pr2_learning_affordances-srv:poses instead.")
  (poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteCartesianIKTrajectory-request>) ostream)
  "Serializes a message object of type '<ExecuteCartesianIKTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteCartesianIKTrajectory-request>) istream)
  "Deserializes a message object of type '<ExecuteCartesianIKTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteCartesianIKTrajectory-request>)))
  "Returns string type for a service object of type '<ExecuteCartesianIKTrajectory-request>"
  "pr2_learning_affordances/ExecuteCartesianIKTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteCartesianIKTrajectory-request)))
  "Returns string type for a service object of type 'ExecuteCartesianIKTrajectory-request"
  "pr2_learning_affordances/ExecuteCartesianIKTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteCartesianIKTrajectory-request>)))
  "Returns md5sum for a message object of type '<ExecuteCartesianIKTrajectory-request>"
  "d904d75e10f01066c74bf87962ffff22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteCartesianIKTrajectory-request)))
  "Returns md5sum for a message object of type 'ExecuteCartesianIKTrajectory-request"
  "d904d75e10f01066c74bf87962ffff22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteCartesianIKTrajectory-request>)))
  "Returns full string definition for message of type '<ExecuteCartesianIKTrajectory-request>"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteCartesianIKTrajectory-request)))
  "Returns full string definition for message of type 'ExecuteCartesianIKTrajectory-request"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteCartesianIKTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteCartesianIKTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteCartesianIKTrajectory-request
    (cl:cons ':header (header msg))
    (cl:cons ':poses (poses msg))
))
;//! \htmlinclude ExecuteCartesianIKTrajectory-response.msg.html

(cl:defclass <ExecuteCartesianIKTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass ExecuteCartesianIKTrajectory-response (<ExecuteCartesianIKTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteCartesianIKTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteCartesianIKTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pr2_learning_affordances-srv:<ExecuteCartesianIKTrajectory-response> is deprecated: use pr2_learning_affordances-srv:ExecuteCartesianIKTrajectory-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ExecuteCartesianIKTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pr2_learning_affordances-srv:success-val is deprecated.  Use pr2_learning_affordances-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteCartesianIKTrajectory-response>) ostream)
  "Serializes a message object of type '<ExecuteCartesianIKTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'success)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'success)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'success)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'success)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteCartesianIKTrajectory-response>) istream)
  "Deserializes a message object of type '<ExecuteCartesianIKTrajectory-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'success)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'success)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'success)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'success)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteCartesianIKTrajectory-response>)))
  "Returns string type for a service object of type '<ExecuteCartesianIKTrajectory-response>"
  "pr2_learning_affordances/ExecuteCartesianIKTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteCartesianIKTrajectory-response)))
  "Returns string type for a service object of type 'ExecuteCartesianIKTrajectory-response"
  "pr2_learning_affordances/ExecuteCartesianIKTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteCartesianIKTrajectory-response>)))
  "Returns md5sum for a message object of type '<ExecuteCartesianIKTrajectory-response>"
  "d904d75e10f01066c74bf87962ffff22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteCartesianIKTrajectory-response)))
  "Returns md5sum for a message object of type 'ExecuteCartesianIKTrajectory-response"
  "d904d75e10f01066c74bf87962ffff22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteCartesianIKTrajectory-response>)))
  "Returns full string definition for message of type '<ExecuteCartesianIKTrajectory-response>"
  (cl:format cl:nil "uint32 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteCartesianIKTrajectory-response)))
  "Returns full string definition for message of type 'ExecuteCartesianIKTrajectory-response"
  (cl:format cl:nil "uint32 success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteCartesianIKTrajectory-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteCartesianIKTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteCartesianIKTrajectory-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExecuteCartesianIKTrajectory)))
  'ExecuteCartesianIKTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExecuteCartesianIKTrajectory)))
  'ExecuteCartesianIKTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteCartesianIKTrajectory)))
  "Returns string type for a service object of type '<ExecuteCartesianIKTrajectory>"
  "pr2_learning_affordances/ExecuteCartesianIKTrajectory")
