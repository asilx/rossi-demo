
(cl:in-package :asdf)

(defsystem "behavior_manager-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Action" :depends-on ("_package_Action"))
    (:file "_package_Action" :depends-on ("_package"))
  ))