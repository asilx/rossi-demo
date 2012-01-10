
(cl:in-package :asdf)

(defsystem "tabletop_2D_segmentation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "Perception2D" :depends-on ("_package_Perception2D"))
    (:file "_package_Perception2D" :depends-on ("_package"))
  ))