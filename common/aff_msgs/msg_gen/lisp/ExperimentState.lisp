; Auto-generated. Do not edit!


(cl:in-package aff_msgs-msg)


;//! \htmlinclude ExperimentState.msg.html

(cl:defclass <ExperimentState> (roslisp-msg-protocol:ros-message)
  ((experiment_state
    :reader experiment_state
    :initarg :experiment_state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ExperimentState (<ExperimentState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExperimentState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExperimentState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aff_msgs-msg:<ExperimentState> is deprecated: use aff_msgs-msg:ExperimentState instead.")))

(cl:ensure-generic-function 'experiment_state-val :lambda-list '(m))
(cl:defmethod experiment_state-val ((m <ExperimentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aff_msgs-msg:experiment_state-val is deprecated.  Use aff_msgs-msg:experiment_state instead.")
  (experiment_state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ExperimentState>)))
    "Constants for message type '<ExperimentState>"
  '((:ASK_FOR_ACTION . 0)
    (:VERIFY_ACTION . 1)
    (:PERCEPTION . 2)
    (:WAIT_FOR_CLEAR_SCENE . 3)
    (:ACTION . 4)
    (:ASK_FOR_EFFECT . 5)
    (:EFFECT . 6)
    (:STANDBY . 7)
    (:LET_HUMAN_ACT . 8))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ExperimentState)))
    "Constants for message type 'ExperimentState"
  '((:ASK_FOR_ACTION . 0)
    (:VERIFY_ACTION . 1)
    (:PERCEPTION . 2)
    (:WAIT_FOR_CLEAR_SCENE . 3)
    (:ACTION . 4)
    (:ASK_FOR_EFFECT . 5)
    (:EFFECT . 6)
    (:STANDBY . 7)
    (:LET_HUMAN_ACT . 8))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExperimentState>) ostream)
  "Serializes a message object of type '<ExperimentState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'experiment_state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExperimentState>) istream)
  "Deserializes a message object of type '<ExperimentState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'experiment_state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExperimentState>)))
  "Returns string type for a message object of type '<ExperimentState>"
  "aff_msgs/ExperimentState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExperimentState)))
  "Returns string type for a message object of type 'ExperimentState"
  "aff_msgs/ExperimentState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExperimentState>)))
  "Returns md5sum for a message object of type '<ExperimentState>"
  "6cc6165af84dbedcd2e4cc0a760f8618")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExperimentState)))
  "Returns md5sum for a message object of type 'ExperimentState"
  "6cc6165af84dbedcd2e4cc0a760f8618")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExperimentState>)))
  "Returns full string definition for message of type '<ExperimentState>"
  (cl:format cl:nil "uint8 ASK_FOR_ACTION = 0 # now what?~%uint8 VERIFY_ACTION = 1 #I'm doing \"this action\"~%uint8 PERCEPTION = 2~%uint8 WAIT_FOR_CLEAR_SCENE = 3 # I'm waiting for a clear scene~%uint8 ACTION = 4~%uint8 ASK_FOR_EFFECT =5 #What has just happened ?~%uint8 EFFECT = 6~%uint8 STANDBY = 7~%uint8 LET_HUMAN_ACT = 8 #human acts on the environment, external motion~%~%uint8 experiment_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExperimentState)))
  "Returns full string definition for message of type 'ExperimentState"
  (cl:format cl:nil "uint8 ASK_FOR_ACTION = 0 # now what?~%uint8 VERIFY_ACTION = 1 #I'm doing \"this action\"~%uint8 PERCEPTION = 2~%uint8 WAIT_FOR_CLEAR_SCENE = 3 # I'm waiting for a clear scene~%uint8 ACTION = 4~%uint8 ASK_FOR_EFFECT =5 #What has just happened ?~%uint8 EFFECT = 6~%uint8 STANDBY = 7~%uint8 LET_HUMAN_ACT = 8 #human acts on the environment, external motion~%~%uint8 experiment_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExperimentState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExperimentState>))
  "Converts a ROS message object to a list"
  (cl:list 'ExperimentState
    (cl:cons ':experiment_state (experiment_state msg))
))
