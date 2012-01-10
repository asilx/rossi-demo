
(cl:in-package :asdf)

(defsystem "feature_factory-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "FeatureCalculation" :depends-on ("_package_FeatureCalculation"))
    (:file "_package_FeatureCalculation" :depends-on ("_package"))
  ))