; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude SpeciesIDRequest.msg.html

(cl:defclass <SpeciesIDRequest> (roslisp-msg-protocol:ros-message)
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
   (image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (regions
    :reader regions
    :initarg :regions
    :type (cl:vector reefbot_msgs-msg:ImageRegion)
   :initform (cl:make-array 0 :element-type 'reefbot_msgs-msg:ImageRegion :initial-element (cl:make-instance 'reefbot_msgs-msg:ImageRegion))))
)

(cl:defclass SpeciesIDRequest (<SpeciesIDRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeciesIDRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeciesIDRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<SpeciesIDRequest> is deprecated: use reefbot_msgs-msg:SpeciesIDRequest instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SpeciesIDRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:header-val is deprecated.  Use reefbot_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image_id-val :lambda-list '(m))
(cl:defmethod image_id-val ((m <SpeciesIDRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:image_id-val is deprecated.  Use reefbot_msgs-msg:image_id instead.")
  (image_id m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <SpeciesIDRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:image-val is deprecated.  Use reefbot_msgs-msg:image instead.")
  (image m))

(cl:ensure-generic-function 'regions-val :lambda-list '(m))
(cl:defmethod regions-val ((m <SpeciesIDRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:regions-val is deprecated.  Use reefbot_msgs-msg:regions instead.")
  (regions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeciesIDRequest>) ostream)
  "Serializes a message object of type '<SpeciesIDRequest>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'image_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'image_id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'regions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'regions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeciesIDRequest>) istream)
  "Deserializes a message object of type '<SpeciesIDRequest>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'image_id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'regions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'regions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'reefbot_msgs-msg:ImageRegion))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeciesIDRequest>)))
  "Returns string type for a message object of type '<SpeciesIDRequest>"
  "reefbot_msgs/SpeciesIDRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeciesIDRequest)))
  "Returns string type for a message object of type 'SpeciesIDRequest"
  "reefbot_msgs/SpeciesIDRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeciesIDRequest>)))
  "Returns md5sum for a message object of type '<SpeciesIDRequest>"
  "0c954188ab1d8afbdf4c56b31e61c471")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeciesIDRequest)))
  "Returns md5sum for a message object of type 'SpeciesIDRequest"
  "0c954188ab1d8afbdf4c56b31e61c471")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeciesIDRequest>)))
  "Returns full string definition for message of type '<SpeciesIDRequest>"
  (cl:format cl:nil "# This message specifies a request to identify the species in a~%# picture.  Only one species will be identified (the one that takes up~%# most of the frame). For best results, use the mask to identify the~%# exact pixels that the individual occupies.~%#~%# If you're using OpenCV to handle images, look up the cv_bridge in ~%# ROS to easily write this message.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%Header header~%~%# Optional Id to identify the image~%uint64 image_id~%~%# Full color image that contains an instance of the species~%sensor_msgs/Image image~%~%# Regions in the image to look for individuals whose species need to~%# be identified.~%ImageRegion[] regions~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: reefbot_msgs/ImageRegion~%# Message that species a region in the image to look at to identify~%# the species.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Bounding box in the image that specifies the region~%sensor_msgs/RegionOfInterest bounding_box~%~%# Binary mask image specifying where the species is in the bounding~%# box. A pixel value of zero means to ignore the pixel. If this image~%# is empty, then the entire box should be used, but if it's not empty,~%# it must be the same size as bbox or else an error will be generated.~%sensor_msgs/Image mask~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeciesIDRequest)))
  "Returns full string definition for message of type 'SpeciesIDRequest"
  (cl:format cl:nil "# This message specifies a request to identify the species in a~%# picture.  Only one species will be identified (the one that takes up~%# most of the frame). For best results, use the mask to identify the~%# exact pixels that the individual occupies.~%#~%# If you're using OpenCV to handle images, look up the cv_bridge in ~%# ROS to easily write this message.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%Header header~%~%# Optional Id to identify the image~%uint64 image_id~%~%# Full color image that contains an instance of the species~%sensor_msgs/Image image~%~%# Regions in the image to look for individuals whose species need to~%# be identified.~%ImageRegion[] regions~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: reefbot_msgs/ImageRegion~%# Message that species a region in the image to look at to identify~%# the species.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Bounding box in the image that specifies the region~%sensor_msgs/RegionOfInterest bounding_box~%~%# Binary mask image specifying where the species is in the bounding~%# box. A pixel value of zero means to ignore the pixel. If this image~%# is empty, then the entire box should be used, but if it's not empty,~%# it must be the same size as bbox or else an error will be generated.~%sensor_msgs/Image mask~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeciesIDRequest>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'regions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeciesIDRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeciesIDRequest
    (cl:cons ':header (header msg))
    (cl:cons ':image_id (image_id msg))
    (cl:cons ':image (image msg))
    (cl:cons ':regions (regions msg))
))
