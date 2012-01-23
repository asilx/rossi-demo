
(cl:in-package :asdf)

(defsystem "aff_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :tabletop_object_detector-msg
)
  :components ((:file "_package")
    (:file "WorkspaceDetection" :depends-on ("_package_WorkspaceDetection"))
    (:file "_package_WorkspaceDetection" :depends-on ("_package"))
    (:file "ObjectOfInterest" :depends-on ("_package_ObjectOfInterest"))
    (:file "_package_ObjectOfInterest" :depends-on ("_package"))
    (:file "Speech" :depends-on ("_package_Speech"))
    (:file "_package_Speech" :depends-on ("_package"))
    (:file "Features" :depends-on ("_package_Features"))
    (:file "_package_Features" :depends-on ("_package"))
    (:file "ExperimentState" :depends-on ("_package_ExperimentState"))
    (:file "_package_ExperimentState" :depends-on ("_package"))
    (:file "ModuleStates" :depends-on ("_package_ModuleStates"))
    (:file "_package_ModuleStates" :depends-on ("_package"))
  ))