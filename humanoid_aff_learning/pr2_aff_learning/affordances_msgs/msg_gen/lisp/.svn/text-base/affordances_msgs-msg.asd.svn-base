
(cl:in-package :asdf)

(defsystem "affordances_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PerceiveRangeData" :depends-on ("_package_PerceiveRangeData"))
    (:file "_package_PerceiveRangeData" :depends-on ("_package"))
  ))