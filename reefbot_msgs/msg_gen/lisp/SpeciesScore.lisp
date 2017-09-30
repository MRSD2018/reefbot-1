; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude SpeciesScore.msg.html

(cl:defclass <SpeciesScore> (roslisp-msg-protocol:ros-message)
  ((species_id
    :reader species_id
    :initarg :species_id
    :type cl:integer
    :initform 0)
   (score
    :reader score
    :initarg :score
    :type cl:float
    :initform 0.0)
   (meta_data
    :reader meta_data
    :initarg :meta_data
    :type cl:string
    :initform ""))
)

(cl:defclass SpeciesScore (<SpeciesScore>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeciesScore>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeciesScore)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<SpeciesScore> is deprecated: use reefbot_msgs-msg:SpeciesScore instead.")))

(cl:ensure-generic-function 'species_id-val :lambda-list '(m))
(cl:defmethod species_id-val ((m <SpeciesScore>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:species_id-val is deprecated.  Use reefbot_msgs-msg:species_id instead.")
  (species_id m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <SpeciesScore>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:score-val is deprecated.  Use reefbot_msgs-msg:score instead.")
  (score m))

(cl:ensure-generic-function 'meta_data-val :lambda-list '(m))
(cl:defmethod meta_data-val ((m <SpeciesScore>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:meta_data-val is deprecated.  Use reefbot_msgs-msg:meta_data instead.")
  (meta_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeciesScore>) ostream)
  "Serializes a message object of type '<SpeciesScore>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'species_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'species_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'species_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'species_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'meta_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'meta_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeciesScore>) istream)
  "Deserializes a message object of type '<SpeciesScore>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'species_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'species_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'species_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'species_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'score) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'meta_data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'meta_data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeciesScore>)))
  "Returns string type for a message object of type '<SpeciesScore>"
  "reefbot_msgs/SpeciesScore")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeciesScore)))
  "Returns string type for a message object of type 'SpeciesScore"
  "reefbot_msgs/SpeciesScore")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeciesScore>)))
  "Returns md5sum for a message object of type '<SpeciesScore>"
  "eefcde7545624590280bada3afd96471")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeciesScore)))
  "Returns md5sum for a message object of type 'SpeciesScore"
  "eefcde7545624590280bada3afd96471")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeciesScore>)))
  "Returns full string definition for message of type '<SpeciesScore>"
  (cl:format cl:nil "# The score of a species identification. The score is dependent on the~%# algorithm being used so it doesn't necessarily have semantic meaning~%# except that a higher score is better.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Unique identifier for the species~%uint32 species_id~%~%# Score for the species. Higher is better~%float32 score~%~%# Optional extra information about the score. This will be algorithm~%# dependent and might be useful for debugging~%string meta_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeciesScore)))
  "Returns full string definition for message of type 'SpeciesScore"
  (cl:format cl:nil "# The score of a species identification. The score is dependent on the~%# algorithm being used so it doesn't necessarily have semantic meaning~%# except that a higher score is better.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Unique identifier for the species~%uint32 species_id~%~%# Score for the species. Higher is better~%float32 score~%~%# Optional extra information about the score. This will be algorithm~%# dependent and might be useful for debugging~%string meta_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeciesScore>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'meta_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeciesScore>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeciesScore
    (cl:cons ':species_id (species_id msg))
    (cl:cons ':score (score msg))
    (cl:cons ':meta_data (meta_data msg))
))
