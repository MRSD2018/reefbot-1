; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude SingleSpeciesId.msg.html

(cl:defclass <SingleSpeciesId> (roslisp-msg-protocol:ros-message)
  ((bounding_box
    :reader bounding_box
    :initarg :bounding_box
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest))
   (best_species
    :reader best_species
    :initarg :best_species
    :type (cl:vector reefbot_msgs-msg:SpeciesScore)
   :initform (cl:make-array 0 :element-type 'reefbot_msgs-msg:SpeciesScore :initial-element (cl:make-instance 'reefbot_msgs-msg:SpeciesScore))))
)

(cl:defclass SingleSpeciesId (<SingleSpeciesId>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SingleSpeciesId>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SingleSpeciesId)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<SingleSpeciesId> is deprecated: use reefbot_msgs-msg:SingleSpeciesId instead.")))

(cl:ensure-generic-function 'bounding_box-val :lambda-list '(m))
(cl:defmethod bounding_box-val ((m <SingleSpeciesId>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:bounding_box-val is deprecated.  Use reefbot_msgs-msg:bounding_box instead.")
  (bounding_box m))

(cl:ensure-generic-function 'best_species-val :lambda-list '(m))
(cl:defmethod best_species-val ((m <SingleSpeciesId>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:best_species-val is deprecated.  Use reefbot_msgs-msg:best_species instead.")
  (best_species m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SingleSpeciesId>) ostream)
  "Serializes a message object of type '<SingleSpeciesId>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bounding_box) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'best_species))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'best_species))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SingleSpeciesId>) istream)
  "Deserializes a message object of type '<SingleSpeciesId>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bounding_box) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'best_species) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'best_species)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'reefbot_msgs-msg:SpeciesScore))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SingleSpeciesId>)))
  "Returns string type for a message object of type '<SingleSpeciesId>"
  "reefbot_msgs/SingleSpeciesId")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SingleSpeciesId)))
  "Returns string type for a message object of type 'SingleSpeciesId"
  "reefbot_msgs/SingleSpeciesId")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SingleSpeciesId>)))
  "Returns md5sum for a message object of type '<SingleSpeciesId>"
  "2e152236e2ac275b6db79639b7684e8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SingleSpeciesId)))
  "Returns md5sum for a message object of type 'SingleSpeciesId"
  "2e152236e2ac275b6db79639b7684e8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SingleSpeciesId>)))
  "Returns full string definition for message of type '<SingleSpeciesId>"
  (cl:format cl:nil "# The best few matches for the species in a single region~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# The bounding box where we found the individual~%sensor_msgs/RegionOfInterest bounding_box~%~%# The most likely species in descending order. This array could be~%# empty if there was no good match.~%SpeciesScore[] best_species~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: reefbot_msgs/SpeciesScore~%# The score of a species identification. The score is dependent on the~%# algorithm being used so it doesn't necessarily have semantic meaning~%# except that a higher score is better.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Unique identifier for the species~%uint32 species_id~%~%# Score for the species. Higher is better~%float32 score~%~%# Optional extra information about the score. This will be algorithm~%# dependent and might be useful for debugging~%string meta_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SingleSpeciesId)))
  "Returns full string definition for message of type 'SingleSpeciesId"
  (cl:format cl:nil "# The best few matches for the species in a single region~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# The bounding box where we found the individual~%sensor_msgs/RegionOfInterest bounding_box~%~%# The most likely species in descending order. This array could be~%# empty if there was no good match.~%SpeciesScore[] best_species~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: reefbot_msgs/SpeciesScore~%# The score of a species identification. The score is dependent on the~%# algorithm being used so it doesn't necessarily have semantic meaning~%# except that a higher score is better.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Unique identifier for the species~%uint32 species_id~%~%# Score for the species. Higher is better~%float32 score~%~%# Optional extra information about the score. This will be algorithm~%# dependent and might be useful for debugging~%string meta_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SingleSpeciesId>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bounding_box))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'best_species) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SingleSpeciesId>))
  "Converts a ROS message object to a list"
  (cl:list 'SingleSpeciesId
    (cl:cons ':bounding_box (bounding_box msg))
    (cl:cons ':best_species (best_species msg))
))
