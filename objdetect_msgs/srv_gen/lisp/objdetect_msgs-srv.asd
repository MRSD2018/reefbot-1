
(cl:in-package :asdf)

(defsystem "objdetect_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :objdetect_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DetectObjectGridService" :depends-on ("_package_DetectObjectGridService"))
    (:file "_package_DetectObjectGridService" :depends-on ("_package"))
    (:file "DetectObjectService" :depends-on ("_package_DetectObjectService"))
    (:file "_package_DetectObjectService" :depends-on ("_package"))
  ))