; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude SpeciesIDResponse.msg.html

(cl:defclass <SpeciesIDResponse> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image_id
    :reader image_id
    :initarg :image_id
    :type cl:integer
    :initform 0)
   (answers
    :reader answers
    :initarg :answers
    :type (cl:vector reefbot_msgs-msg:SingleSpeciesId)
   :initform (cl:make-array 0 :element-type 'reefbot_msgs-msg:SingleSpeciesId :initial-element (cl:make-instance 'reefbot_msgs-msg:SingleSpeciesId))))
)

(cl:defclass SpeciesIDResponse (<SpeciesIDResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeciesIDResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeciesIDResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<SpeciesIDResponse> is deprecated: use reefbot_msgs-msg:SpeciesIDResponse instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SpeciesIDResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:header-val is deprecated.  Use reefbot_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image_id-val :lambda-list '(m))
(cl:defmethod image_id-val ((m <SpeciesIDResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:image_id-val is deprecated.  Use reefbot_msgs-msg:image_id instead.")
  (image_id m))

(cl:ensure-generic-function 'answers-val :lambda-list '(m))
(cl:defmethod answers-val ((m <SpeciesIDResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:answers-val is deprecated.  Use reefbot_msgs-msg:answers instead.")
  (answers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeciesIDResponse>) ostream)
  "Serializes a message object of type '<SpeciesIDResponse>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'image_id)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'answers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'answers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeciesIDResponse>) istream)
  "Deserializes a message object of type '<SpeciesIDResponse>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'answers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'answers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'reefbot_msgs-msg:SingleSpeciesId))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeciesIDResponse>)))
  "Returns string type for a message object of type '<SpeciesIDResponse>"
  "reefbot_msgs/SpeciesIDResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeciesIDResponse)))
  "Returns string type for a message object of type 'SpeciesIDResponse"
  "reefbot_msgs/SpeciesIDResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeciesIDResponse>)))
  "Returns md5sum for a message object of type '<SpeciesIDResponse>"
  "6a90c0010a887f2021d1e9fe27d4c819")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeciesIDResponse)))
  "Returns md5sum for a message object of type 'SpeciesIDResponse"
  "6a90c0010a887f2021d1e9fe27d4c819")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeciesIDResponse>)))
  "Returns full string definition for message of type '<SpeciesIDResponse>"
  (cl:format cl:nil "# Response to a SpeciesIDRequest that tells the user the best guess of~%# what species were found.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%Header header~%~%# Optional Id to identify the image~%uint64 image_id~%~%# Answers, one for each region in the Request and in the same order.~%SingleSpeciesId[] answers~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: reefbot_msgs/SingleSpeciesId~%# The best few matches for the species in a single region~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# The bounding box where we found the individual~%sensor_msgs/RegionOfInterest bounding_box~%~%# The most likely species in descending order. This array could be~%# empty if there was no good match.~%SpeciesScore[] best_species~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: reefbot_msgs/SpeciesScore~%# The score of a species identification. The score is dependent on the~%# algorithm being used so it doesn't necessarily have semantic meaning~%# except that a higher score is better.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Unique identifier for the species~%uint32 species_id~%~%# Score for the species. Higher is better~%float32 score~%~%# Optional extra information about the score. This will be algorithm~%# dependent and might be useful for debugging~%string meta_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeciesIDResponse)))
  "Returns full string definition for message of type 'SpeciesIDResponse"
  (cl:format cl:nil "# Response to a SpeciesIDRequest that tells the user the best guess of~%# what species were found.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%Header header~%~%# Optional Id to identify the image~%uint64 image_id~%~%# Answers, one for each region in the Request and in the same order.~%SingleSpeciesId[] answers~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: reefbot_msgs/SingleSpeciesId~%# The best few matches for the species in a single region~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# The bounding box where we found the individual~%sensor_msgs/RegionOfInterest bounding_box~%~%# The most likely species in descending order. This array could be~%# empty if there was no good match.~%SpeciesScore[] best_species~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: reefbot_msgs/SpeciesScore~%# The score of a species identification. The score is dependent on the~%# algorithm being used so it doesn't necessarily have semantic meaning~%# except that a higher score is better.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Unique identifier for the species~%uint32 species_id~%~%# Score for the species. Higher is better~%float32 score~%~%# Optional extra information about the score. This will be algorithm~%# dependent and might be useful for debugging~%string meta_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeciesIDResponse>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'answers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeciesIDResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeciesIDResponse
    (cl:cons ':header (header msg))
    (cl:cons ':image_id (image_id msg))
    (cl:cons ':answers (answers msg))
))
