; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude ImageRegion.msg.html

(cl:defclass <ImageRegion> (roslisp-msg-protocol:ros-message)
  ((bounding_box
    :reader bounding_box
    :initarg :bounding_box
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest))
   (mask
    :reader mask
    :initarg :mask
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass ImageRegion (<ImageRegion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageRegion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageRegion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<ImageRegion> is deprecated: use reefbot_msgs-msg:ImageRegion instead.")))

(cl:ensure-generic-function 'bounding_box-val :lambda-list '(m))
(cl:defmethod bounding_box-val ((m <ImageRegion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:bounding_box-val is deprecated.  Use reefbot_msgs-msg:bounding_box instead.")
  (bounding_box m))

(cl:ensure-generic-function 'mask-val :lambda-list '(m))
(cl:defmethod mask-val ((m <ImageRegion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:mask-val is deprecated.  Use reefbot_msgs-msg:mask instead.")
  (mask m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageRegion>) ostream)
  "Serializes a message object of type '<ImageRegion>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bounding_box) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mask) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageRegion>) istream)
  "Deserializes a message object of type '<ImageRegion>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bounding_box) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mask) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageRegion>)))
  "Returns string type for a message object of type '<ImageRegion>"
  "reefbot_msgs/ImageRegion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageRegion)))
  "Returns string type for a message object of type 'ImageRegion"
  "reefbot_msgs/ImageRegion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageRegion>)))
  "Returns md5sum for a message object of type '<ImageRegion>"
  "65cc1a85d539c02ff4e503921c8e033b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageRegion)))
  "Returns md5sum for a message object of type 'ImageRegion"
  "65cc1a85d539c02ff4e503921c8e033b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageRegion>)))
  "Returns full string definition for message of type '<ImageRegion>"
  (cl:format cl:nil "# Message that species a region in the image to look at to identify~%# the species.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Bounding box in the image that specifies the region~%sensor_msgs/RegionOfInterest bounding_box~%~%# Binary mask image specifying where the species is in the bounding~%# box. A pixel value of zero means to ignore the pixel. If this image~%# is empty, then the entire box should be used, but if it's not empty,~%# it must be the same size as bbox or else an error will be generated.~%sensor_msgs/Image mask~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageRegion)))
  "Returns full string definition for message of type 'ImageRegion"
  (cl:format cl:nil "# Message that species a region in the image to look at to identify~%# the species.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Bounding box in the image that specifies the region~%sensor_msgs/RegionOfInterest bounding_box~%~%# Binary mask image specifying where the species is in the bounding~%# box. A pixel value of zero means to ignore the pixel. If this image~%# is empty, then the entire box should be used, but if it's not empty,~%# it must be the same size as bbox or else an error will be generated.~%sensor_msgs/Image mask~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageRegion>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bounding_box))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mask))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageRegion>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageRegion
    (cl:cons ':bounding_box (bounding_box msg))
    (cl:cons ':mask (mask msg))
))
