
(cl:in-package :asdf)

(defsystem "objdetect_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Grid" :depends-on ("_package_Grid"))
    (:file "_package_Grid" :depends-on ("_package"))
    (:file "DetectGridScores" :depends-on ("_package_DetectGridScores"))
    (:file "_package_DetectGridScores" :depends-on ("_package"))
    (:file "Detection" :depends-on ("_package_Detection"))
    (:file "_package_Detection" :depends-on ("_package"))
    (:file "DetectObject" :depends-on ("_package_DetectObject"))
    (:file "_package_DetectObject" :depends-on ("_package"))
    (:file "DetectionArray" :depends-on ("_package_DetectionArray"))
    (:file "_package_DetectionArray" :depends-on ("_package"))
    (:file "Mask" :depends-on ("_package_Mask"))
    (:file "_package_Mask" :depends-on ("_package"))
    (:file "DetectObjectGrid" :depends-on ("_package_DetectObjectGrid"))
    (:file "_package_DetectObjectGrid" :depends-on ("_package"))
  ))