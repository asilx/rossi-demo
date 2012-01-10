; Auto-generated. Do not edit!


(cl:in-package aff_msgs-msg)


;//! \htmlinclude ModuleStates.msg.html

(cl:defclass <ModuleStates> (roslisp-msg-protocol:ros-message)
  ((workspace_detector
    :reader workspace_detector
    :initarg :workspace_detector
    :type cl:boolean
    :initform cl:nil)
   (behavior_manager
    :reader behavior_manager
    :initarg :behavior_manager
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ModuleStates (<ModuleStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModuleStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModuleStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aff_msgs-msg:<ModuleStates> is deprecated: use aff_msgs-msg:ModuleStates instead.")))

(cl:ensure-generic-function 'workspace_detector-val :lambda-list '(m))
(cl:defmethod workspace_detector-val ((m <ModuleStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:workspace_detector-val is deprecated.  Use aff_msgs-msg:workspace_detector instead.")
  (workspace_detector m))

(cl:ensure-generic-function 'behavior_manager-val :lambda-list '(m))
(cl:defmethod behavior_manager-val ((m <ModuleStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:behavior_manager-val is deprecated.  Use aff_msgs-msg:behavior_manager instead.")
  (behavior_manager m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModuleStates>) ostream)
  "Serializes a message object of type '<ModuleStates>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'workspace_detector) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'behavior_manager) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModuleStates>) istream)
  "Deserializes a message object of type '<ModuleStates>"
    (cl:setf (cl:slot-value msg 'workspace_detector) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'behavior_manager) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModuleStates>)))
  "Returns string type for a message object of type '<ModuleStates>"
  "aff_msgs/ModuleStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModuleStates)))
  "Returns string type for a message object of type 'ModuleStates"
  "aff_msgs/ModuleStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModuleStates>)))
  "Returns md5sum for a message object of type '<ModuleStates>"
  "5db198112592b255720236133b837d16")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModuleStates)))
  "Returns md5sum for a message object of type 'ModuleStates"
  "5db198112592b255720236133b837d16")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModuleStates>)))
  "Returns full string definition for message of type '<ModuleStates>"
  (cl:format cl:nil "# This message is sent to all of the nodes to synchronize sensori-data acquisition,~%# action, observation and teleoperation stuff.~%~%bool workspace_detector~%bool behavior_manager~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModuleStates)))
  "Returns full string definition for message of type 'ModuleStates"
  (cl:format cl:nil "# This message is sent to all of the nodes to synchronize sensori-data acquisition,~%# action, observation and teleoperation stuff.~%~%bool workspace_detector~%bool behavior_manager~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModuleStates>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModuleStates>))
  "Converts a ROS message object to a list"
  (cl:list 'ModuleStates
    (cl:cons ':workspace_detector (workspace_detector msg))
    (cl:cons ':behavior_manager (behavior_manager msg))
))
