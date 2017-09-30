; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude LogVideo.msg.html

(cl:defclass <LogVideo> (roslisp-msg-protocol:ros-message)
  ((rtp_ip
    :reader rtp_ip
    :initarg :rtp_ip
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (rtp_port
    :reader rtp_port
    :initarg :rtp_port
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass LogVideo (<LogVideo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LogVideo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LogVideo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<LogVideo> is deprecated: use reefbot_msgs-msg:LogVideo instead.")))

(cl:ensure-generic-function 'rtp_ip-val :lambda-list '(m))
(cl:defmethod rtp_ip-val ((m <LogVideo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:rtp_ip-val is deprecated.  Use reefbot_msgs-msg:rtp_ip instead.")
  (rtp_ip m))

(cl:ensure-generic-function 'rtp_port-val :lambda-list '(m))
(cl:defmethod rtp_port-val ((m <LogVideo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:rtp_port-val is deprecated.  Use reefbot_msgs-msg:rtp_port instead.")
  (rtp_port m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <LogVideo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:duration-val is deprecated.  Use reefbot_msgs-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LogVideo>) ostream)
  "Serializes a message object of type '<LogVideo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rtp_ip) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rtp_port) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LogVideo>) istream)
  "Deserializes a message object of type '<LogVideo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rtp_ip) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rtp_port) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LogVideo>)))
  "Returns string type for a message object of type '<LogVideo>"
  "reefbot_msgs/LogVideo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LogVideo)))
  "Returns string type for a message object of type 'LogVideo"
  "reefbot_msgs/LogVideo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LogVideo>)))
  "Returns md5sum for a message object of type '<LogVideo>"
  "138afce7a4d9872b7ca317a623a839c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LogVideo)))
  "Returns md5sum for a message object of type 'LogVideo"
  "138afce7a4d9872b7ca317a623a839c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LogVideo>)))
  "Returns full string definition for message of type '<LogVideo>"
  (cl:format cl:nil "# A message that says to start logging some video~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: Oct 2010~%~%# The ip of the broadcast stream~%std_msgs/String rtp_ip~%~%# The port of the broadcast stream~%std_msgs/String rtp_port~%~%# How long to log the video in seconds. 0 means go until we die.~%float64 duration~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LogVideo)))
  "Returns full string definition for message of type 'LogVideo"
  (cl:format cl:nil "# A message that says to start logging some video~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: Oct 2010~%~%# The ip of the broadcast stream~%std_msgs/String rtp_ip~%~%# The port of the broadcast stream~%std_msgs/String rtp_port~%~%# How long to log the video in seconds. 0 means go until we die.~%float64 duration~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LogVideo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rtp_ip))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rtp_port))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LogVideo>))
  "Converts a ROS message object to a list"
  (cl:list 'LogVideo
    (cl:cons ':rtp_ip (rtp_ip msg))
    (cl:cons ':rtp_port (rtp_port msg))
    (cl:cons ':duration (duration msg))
))
