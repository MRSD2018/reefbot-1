; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude RobotHealth.msg.html

(cl:defclass <RobotHealth> (roslisp-msg-protocol:ros-message)
  ((voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0)
   (router_comms_ok
    :reader router_comms_ok
    :initarg :router_comms_ok
    :type cl:boolean
    :initform cl:nil)
   (robot_comms_ok
    :reader robot_comms_ok
    :initarg :robot_comms_ok
    :type cl:boolean
    :initform cl:nil)
   (left_motor_ok
    :reader left_motor_ok
    :initarg :left_motor_ok
    :type cl:boolean
    :initform cl:nil)
   (right_motor_ok
    :reader right_motor_ok
    :initarg :right_motor_ok
    :type cl:boolean
    :initform cl:nil)
   (vertical_motor_ok
    :reader vertical_motor_ok
    :initarg :vertical_motor_ok
    :type cl:boolean
    :initform cl:nil)
   (depth_sensor_ok
    :reader depth_sensor_ok
    :initarg :depth_sensor_ok
    :type cl:boolean
    :initform cl:nil)
   (heading_sensor_ok
    :reader heading_sensor_ok
    :initarg :heading_sensor_ok
    :type cl:boolean
    :initform cl:nil)
   (robot_error_code
    :reader robot_error_code
    :initarg :robot_error_code
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotHealth (<RobotHealth>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotHealth>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotHealth)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<RobotHealth> is deprecated: use reefbot_msgs-msg:RobotHealth instead.")))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <RobotHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:voltage-val is deprecated.  Use reefbot_msgs-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'router_comms_ok-val :lambda-list '(m))
(cl:defmethod router_comms_ok-val ((m <RobotHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:router_comms_ok-val is deprecated.  Use reefbot_msgs-msg:router_comms_ok instead.")
  (router_comms_ok m))

(cl:ensure-generic-function 'robot_comms_ok-val :lambda-list '(m))
(cl:defmethod robot_comms_ok-val ((m <RobotHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:robot_comms_ok-val is deprecated.  Use reefbot_msgs-msg:robot_comms_ok instead.")
  (robot_comms_ok m))

(cl:ensure-generic-function 'left_motor_ok-val :lambda-list '(m))
(cl:defmethod left_motor_ok-val ((m <RobotHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:left_motor_ok-val is deprecated.  Use reefbot_msgs-msg:left_motor_ok instead.")
  (left_motor_ok m))

(cl:ensure-generic-function 'right_motor_ok-val :lambda-list '(m))
(cl:defmethod right_motor_ok-val ((m <RobotHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:right_motor_ok-val is deprecated.  Use reefbot_msgs-msg:right_motor_ok instead.")
  (right_motor_ok m))

(cl:ensure-generic-function 'vertical_motor_ok-val :lambda-list '(m))
(cl:defmethod vertical_motor_ok-val ((m <RobotHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:vertical_motor_ok-val is deprecated.  Use reefbot_msgs-msg:vertical_motor_ok instead.")
  (vertical_motor_ok m))

(cl:ensure-generic-function 'depth_sensor_ok-val :lambda-list '(m))
(cl:defmethod depth_sensor_ok-val ((m <RobotHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:depth_sensor_ok-val is deprecated.  Use reefbot_msgs-msg:depth_sensor_ok instead.")
  (depth_sensor_ok m))

(cl:ensure-generic-function 'heading_sensor_ok-val :lambda-list '(m))
(cl:defmethod heading_sensor_ok-val ((m <RobotHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:heading_sensor_ok-val is deprecated.  Use reefbot_msgs-msg:heading_sensor_ok instead.")
  (heading_sensor_ok m))

(cl:ensure-generic-function 'robot_error_code-val :lambda-list '(m))
(cl:defmethod robot_error_code-val ((m <RobotHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:robot_error_code-val is deprecated.  Use reefbot_msgs-msg:robot_error_code instead.")
  (robot_error_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotHealth>) ostream)
  "Serializes a message object of type '<RobotHealth>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'router_comms_ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'robot_comms_ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_motor_ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_motor_ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'vertical_motor_ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'depth_sensor_ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'heading_sensor_ok) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'robot_error_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotHealth>) istream)
  "Deserializes a message object of type '<RobotHealth>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'router_comms_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'robot_comms_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'left_motor_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_motor_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'vertical_motor_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'depth_sensor_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'heading_sensor_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_error_code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotHealth>)))
  "Returns string type for a message object of type '<RobotHealth>"
  "reefbot_msgs/RobotHealth")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotHealth)))
  "Returns string type for a message object of type 'RobotHealth"
  "reefbot_msgs/RobotHealth")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotHealth>)))
  "Returns md5sum for a message object of type '<RobotHealth>"
  "b1b87a859cb91e9334be183880bf9ecd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotHealth)))
  "Returns md5sum for a message object of type 'RobotHealth"
  "b1b87a859cb91e9334be183880bf9ecd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotHealth>)))
  "Returns full string definition for message of type '<RobotHealth>"
  (cl:format cl:nil "# Message that specifies the robot's health as known by the Robot Controller~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: July 2010~%~%# Voltage going to the robot in Volts. Note, as of initial deployment,~%# this will probably be either 48V or 0V because we don't actually~%# have a volt meter.~%float32 voltage~%~%# Do we have a comms link to the router~%bool router_comms_ok~%~%# Do we have a comms link to the robot~%bool robot_comms_ok~%~%# Are the various motors operational~%bool left_motor_ok~%bool right_motor_ok~%bool vertical_motor_ok~%~%# Are we getting valid readings from the depth sensor~%bool depth_sensor_ok~%~%# Are we getting valid readings from the heading sensor~%bool heading_sensor_ok~%~%# Error code reported by the robot~%int32 robot_error_code~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotHealth)))
  "Returns full string definition for message of type 'RobotHealth"
  (cl:format cl:nil "# Message that specifies the robot's health as known by the Robot Controller~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: July 2010~%~%# Voltage going to the robot in Volts. Note, as of initial deployment,~%# this will probably be either 48V or 0V because we don't actually~%# have a volt meter.~%float32 voltage~%~%# Do we have a comms link to the router~%bool router_comms_ok~%~%# Do we have a comms link to the robot~%bool robot_comms_ok~%~%# Are the various motors operational~%bool left_motor_ok~%bool right_motor_ok~%bool vertical_motor_ok~%~%# Are we getting valid readings from the depth sensor~%bool depth_sensor_ok~%~%# Are we getting valid readings from the heading sensor~%bool heading_sensor_ok~%~%# Error code reported by the robot~%int32 robot_error_code~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotHealth>))
  (cl:+ 0
     4
     1
     1
     1
     1
     1
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotHealth>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotHealth
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':router_comms_ok (router_comms_ok msg))
    (cl:cons ':robot_comms_ok (robot_comms_ok msg))
    (cl:cons ':left_motor_ok (left_motor_ok msg))
    (cl:cons ':right_motor_ok (right_motor_ok msg))
    (cl:cons ':vertical_motor_ok (vertical_motor_ok msg))
    (cl:cons ':depth_sensor_ok (depth_sensor_ok msg))
    (cl:cons ':heading_sensor_ok (heading_sensor_ok msg))
    (cl:cons ':robot_error_code (robot_error_code msg))
))
