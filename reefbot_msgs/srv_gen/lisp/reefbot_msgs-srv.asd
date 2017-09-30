
(cl:in-package :asdf)

(defsystem "reefbot_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :reefbot_msgs-msg
)
  :components ((:file "_package")
    (:file "FindSpecies" :depends-on ("_package_FindSpecies"))
    (:file "_package_FindSpecies" :depends-on ("_package"))
  ))