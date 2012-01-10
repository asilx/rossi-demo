
(cl:in-package :asdf)

(defsystem "pr2_learning_affordances-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ExecuteCartesianIKTrajectory" :depends-on ("_package_ExecuteCartesianIKTrajectory"))
    (:file "_package_ExecuteCartesianIKTrajectory" :depends-on ("_package"))
  ))