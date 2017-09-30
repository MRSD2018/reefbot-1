; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude UserSpeciesSelection.msg.html

(cl:defclass <UserSpeciesSelection> (roslisp-msg-protocol:ros-message)
  ((image_id
    :reader image_id
    :initarg :image_id
    :type cl:integer
    :initform 0)
   (image_path
    :reader image_path
    :initarg :image_path
    :type cl:string
    :initform "")
   (species_id
    :reader species_id
    :initarg :species_id
    :type cl:integer
    :initform 0))
)

(cl:defclass UserSpeciesSelection (<UserSpeciesSelection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UserSpeciesSelection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UserSpeciesSelection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<UserSpeciesSelection> is deprecated: use reefbot_msgs-msg:UserSpeciesSelection instead.")))

(cl:ensure-generic-function 'image_id-val :lambda-list '(m))
(cl:defmethod image_id-val ((m <UserSpeciesSelection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:image_id-val is deprecated.  Use reefbot_msgs-msg:image_id instead.")
  (image_id m))

(cl:ensure-generic-function 'image_path-val :lambda-list '(m))
(cl:defmethod image_path-val ((m <UserSpeciesSelection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:image_path-val is deprecated.  Use reefbot_msgs-msg:image_path instead.")
  (image_path m))

(cl:ensure-generic-function 'species_id-val :lambda-list '(m))
(cl:defmethod species_id-val ((m <UserSpeciesSelection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:species_id-val is deprecated.  Use reefbot_msgs-msg:species_id instead.")
  (species_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UserSpeciesSelection>) ostream)
  "Serializes a message object of type '<UserSpeciesSelection>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'image_id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'image_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'image_path))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'species_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'species_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'species_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'species_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UserSpeciesSelection>) istream)
  "Deserializes a message object of type '<UserSpeciesSelection>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'image_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'image_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'species_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'species_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'species_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'species_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UserSpeciesSelection>)))
  "Returns string type for a message object of type '<UserSpeciesSelection>"
  "reefbot_msgs/UserSpeciesSelection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UserSpeciesSelection)))
  "Returns string type for a message object of type 'UserSpeciesSelection"
  "reefbot_msgs/UserSpeciesSelection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UserSpeciesSelection>)))
  "Returns md5sum for a message object of type '<UserSpeciesSelection>"
  "f8c0a6a30877a6a6f7257570567384f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UserSpeciesSelection)))
  "Returns md5sum for a message object of type 'UserSpeciesSelection"
  "f8c0a6a30877a6a6f7257570567384f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UserSpeciesSelection>)))
  "Returns full string definition for message of type '<UserSpeciesSelection>"
  (cl:format cl:nil "# Message that specifies that the user had tagged the species for an image~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: Sept 2010~%~%uint64 image_id~%~%string image_path~%~%uint32 species_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UserSpeciesSelection)))
  "Returns full string definition for message of type 'UserSpeciesSelection"
  (cl:format cl:nil "# Message that specifies that the user had tagged the species for an image~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: Sept 2010~%~%uint64 image_id~%~%string image_path~%~%uint32 species_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UserSpeciesSelection>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'image_path))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UserSpeciesSelection>))
  "Converts a ROS message object to a list"
  (cl:list 'UserSpeciesSelection
    (cl:cons ':image_id (image_id msg))
    (cl:cons ':image_path (image_path msg))
    (cl:cons ':species_id (species_id msg))
))
