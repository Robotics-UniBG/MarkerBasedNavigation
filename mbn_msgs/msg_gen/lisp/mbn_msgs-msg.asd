
(cl:in-package :asdf)

(defsystem "mbn_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MarkersIDs" :depends-on ("_package_MarkersIDs"))
    (:file "_package_MarkersIDs" :depends-on ("_package"))
    (:file "MarkersPoses" :depends-on ("_package_MarkersPoses"))
    (:file "_package_MarkersPoses" :depends-on ("_package"))
    (:file "MarkerPose" :depends-on ("_package_MarkerPose"))
    (:file "_package_MarkerPose" :depends-on ("_package"))
  ))