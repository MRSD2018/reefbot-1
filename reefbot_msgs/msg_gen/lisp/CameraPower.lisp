; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude CameraPower.msg.html

(cl:defclass <CameraPower> (roslisp-msg-protocol:ros-message)
  ((turn_camera_on
    :reader turn_camera_on
    :initarg :turn_camera_on
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CameraPower (<CameraPower>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraPower>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraPower)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<CameraPower> is deprecated: use reefbot_msgs-msg:CameraPower instead.")))

(cl:ensure-generic-function 'turn_camera_on-val :lambda-list '(m))
(cl:defmethod turn_camera_on-val ((m <CameraPower>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:turn_camera_on-val is deprecated.  Use reefbot_msgs-msg:turn_camera_on instead.")
  (turn_camera_on m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraPower>) ostream)
  "Serializes a message object of type '<CameraPower>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'turn_camera_on) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraPower>) istream)
  "Deserializes a message object of type '<CameraPower>"
    (cl:setf (cl:slot-value msg 'turn_camera_on) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraPower>)))
  "Returns string type for a message object of type '<CameraPower>"
  "reefbot_msgs/CameraPower")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraPower)))
  "Returns string type for a message object of type 'CameraPower"
  "reefbot_msgs/CameraPower")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraPower>)))
  "Returns md5sum for a message object of type '<CameraPower>"
  "dcac9ec93e4a365c3d8660001ab16af4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraPower)))
  "Returns md5sum for a message object of type 'CameraPower"
  "dcac9ec93e4a365c3d8660001ab16af4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraPower>)))
  "Returns full string definition for message of type '<CameraPower>"
  (cl:format cl:nil "# A message to toggle the camera power manually~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: Sept 2010~%~%bool turn_camera_on~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraPower)))
  "Returns full string definition for message of type 'CameraPower"
  (cl:format cl:nil "# A message to toggle the camera power manually~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: Sept 2010~%~%bool turn_camera_on~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraPower>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraPower>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraPower
    (cl:cons ':turn_camera_on (turn_camera_on msg))
))
