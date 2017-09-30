; Auto-generated. Do not edit!


(cl:in-package objdetect_msgs-msg)


;//! \htmlinclude Mask.msg.html

(cl:defclass <Mask> (roslisp-msg-protocol:ros-message)
  ((roi
    :reader roi
    :initarg :roi
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest))
   (mask
    :reader mask
    :initarg :mask
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass Mask (<Mask>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Mask>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Mask)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objdetect_msgs-msg:<Mask> is deprecated: use objdetect_msgs-msg:Mask instead.")))

(cl:ensure-generic-function 'roi-val :lambda-list '(m))
(cl:defmethod roi-val ((m <Mask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:roi-val is deprecated.  Use objdetect_msgs-msg:roi instead.")
  (roi m))

(cl:ensure-generic-function 'mask-val :lambda-list '(m))
(cl:defmethod mask-val ((m <Mask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:mask-val is deprecated.  Use objdetect_msgs-msg:mask instead.")
  (mask m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Mask>) ostream)
  "Serializes a message object of type '<Mask>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'roi) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mask) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Mask>) istream)
  "Deserializes a message object of type '<Mask>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'roi) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mask) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Mask>)))
  "Returns string type for a message object of type '<Mask>"
  "objdetect_msgs/Mask")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Mask)))
  "Returns string type for a message object of type 'Mask"
  "objdetect_msgs/Mask")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Mask>)))
  "Returns md5sum for a message object of type '<Mask>"
  "ecbf10b456dc7d2982ac745b3ea8ef9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Mask)))
  "Returns md5sum for a message object of type 'Mask"
  "ecbf10b456dc7d2982ac745b3ea8ef9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Mask>)))
  "Returns full string definition for message of type '<Mask>"
  (cl:format cl:nil "# this message is used to mark where an object is present in an image~%# this can be done either by a roi region on the image (less precise)~%# or a mask (more precise)~%~%sensor_msgs/RegionOfInterest roi~%~%# in the case when mask is used, 'roi' specifies the image region and 'mask'~%# (which should be of the same size) a binary mask in that region~%sensor_msgs/Image mask~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Mask)))
  "Returns full string definition for message of type 'Mask"
  (cl:format cl:nil "# this message is used to mark where an object is present in an image~%# this can be done either by a roi region on the image (less precise)~%# or a mask (more precise)~%~%sensor_msgs/RegionOfInterest roi~%~%# in the case when mask is used, 'roi' specifies the image region and 'mask'~%# (which should be of the same size) a binary mask in that region~%sensor_msgs/Image mask~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Mask>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'roi))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mask))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Mask>))
  "Converts a ROS message object to a list"
  (cl:list 'Mask
    (cl:cons ':roi (roi msg))
    (cl:cons ':mask (mask msg))
))
