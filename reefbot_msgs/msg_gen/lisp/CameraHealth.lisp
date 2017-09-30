; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude CameraHealth.msg.html

(cl:defclass <CameraHealth> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ping_ok
    :reader ping_ok
    :initarg :ping_ok
    :type cl:boolean
    :initform cl:nil)
   (http_ping_ok
    :reader http_ping_ok
    :initarg :http_ping_ok
    :type cl:boolean
    :initform cl:nil)
   (error_code
    :reader error_code
    :initarg :error_code
    :type cl:integer
    :initform 0))
)

(cl:defclass CameraHealth (<CameraHealth>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraHealth>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraHealth)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<CameraHealth> is deprecated: use reefbot_msgs-msg:CameraHealth instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CameraHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:header-val is deprecated.  Use reefbot_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ping_ok-val :lambda-list '(m))
(cl:defmethod ping_ok-val ((m <CameraHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:ping_ok-val is deprecated.  Use reefbot_msgs-msg:ping_ok instead.")
  (ping_ok m))

(cl:ensure-generic-function 'http_ping_ok-val :lambda-list '(m))
(cl:defmethod http_ping_ok-val ((m <CameraHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:http_ping_ok-val is deprecated.  Use reefbot_msgs-msg:http_ping_ok instead.")
  (http_ping_ok m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <CameraHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:error_code-val is deprecated.  Use reefbot_msgs-msg:error_code instead.")
  (error_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraHealth>) ostream)
  "Serializes a message object of type '<CameraHealth>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ping_ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'http_ping_ok) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'error_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraHealth>) istream)
  "Deserializes a message object of type '<CameraHealth>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'ping_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'http_ping_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraHealth>)))
  "Returns string type for a message object of type '<CameraHealth>"
  "reefbot_msgs/CameraHealth")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraHealth)))
  "Returns string type for a message object of type 'CameraHealth"
  "reefbot_msgs/CameraHealth")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraHealth>)))
  "Returns md5sum for a message object of type '<CameraHealth>"
  "45c3a1ccc9e7dc2f5a93095c761036c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraHealth)))
  "Returns md5sum for a message object of type 'CameraHealth"
  "45c3a1ccc9e7dc2f5a93095c761036c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraHealth>)))
  "Returns full string definition for message of type '<CameraHealth>"
  (cl:format cl:nil "# Message that specifies the camera's health~%#~%# Author: Mark Desnoyer~%# Date: July 2010~%~%Header header~%~%# Can we ping the camera?~%bool ping_ok~%~%# Can we connect to the http server on the camera~%bool http_ping_ok~%~%# Error code reported by the camera. 0 means that there is no~%# error. See CameraWatchdog.py for error codes.~%int32 error_code~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraHealth)))
  "Returns full string definition for message of type 'CameraHealth"
  (cl:format cl:nil "# Message that specifies the camera's health~%#~%# Author: Mark Desnoyer~%# Date: July 2010~%~%Header header~%~%# Can we ping the camera?~%bool ping_ok~%~%# Can we connect to the http server on the camera~%bool http_ping_ok~%~%# Error code reported by the camera. 0 means that there is no~%# error. See CameraWatchdog.py for error codes.~%int32 error_code~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraHealth>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraHealth>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraHealth
    (cl:cons ':header (header msg))
    (cl:cons ':ping_ok (ping_ok msg))
    (cl:cons ':http_ping_ok (http_ping_ok msg))
    (cl:cons ':error_code (error_code msg))
))
