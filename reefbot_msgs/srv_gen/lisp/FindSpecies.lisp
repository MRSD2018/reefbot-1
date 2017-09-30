; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-srv)


;//! \htmlinclude FindSpecies-request.msg.html

(cl:defclass <FindSpecies-request> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type reefbot_msgs-msg:ImageCaptured
    :initform (cl:make-instance 'reefbot_msgs-msg:ImageCaptured)))
)

(cl:defclass FindSpecies-request (<FindSpecies-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindSpecies-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindSpecies-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-srv:<FindSpecies-request> is deprecated: use reefbot_msgs-srv:FindSpecies-request instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <FindSpecies-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-srv:image-val is deprecated.  Use reefbot_msgs-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindSpecies-request>) ostream)
  "Serializes a message object of type '<FindSpecies-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindSpecies-request>) istream)
  "Deserializes a message object of type '<FindSpecies-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindSpecies-request>)))
  "Returns string type for a service object of type '<FindSpecies-request>"
  "reefbot_msgs/FindSpeciesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindSpecies-request)))
  "Returns string type for a service object of type 'FindSpecies-request"
  "reefbot_msgs/FindSpeciesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindSpecies-request>)))
  "Returns md5sum for a message object of type '<FindSpecies-request>"
  "7da9366c50b21c9cef266c42d369b77c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindSpecies-request)))
  "Returns md5sum for a message object of type 'FindSpecies-request"
  "7da9366c50b21c9cef266c42d369b77c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindSpecies-request>)))
  "Returns full string definition for message of type '<FindSpecies-request>"
  (cl:format cl:nil "ImageCaptured image~%~%================================================================================~%MSG: reefbot_msgs/ImageCaptured~%# This message specifies that a still image was captured by the system.~%#~%# If you're using OpenCV to handle images, look up the cv_bridge in ~%# ROS to easily write/read this message.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: Sept 2010~%~%Header header~%~%# Optional Id to identify the image~%uint64 image_id~%~%# Full color image that was captured~%sensor_msgs/Image image~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindSpecies-request)))
  "Returns full string definition for message of type 'FindSpecies-request"
  (cl:format cl:nil "ImageCaptured image~%~%================================================================================~%MSG: reefbot_msgs/ImageCaptured~%# This message specifies that a still image was captured by the system.~%#~%# If you're using OpenCV to handle images, look up the cv_bridge in ~%# ROS to easily write/read this message.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: Sept 2010~%~%Header header~%~%# Optional Id to identify the image~%uint64 image_id~%~%# Full color image that was captured~%sensor_msgs/Image image~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindSpecies-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindSpecies-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FindSpecies-request
    (cl:cons ':image (image msg))
))
;//! \htmlinclude FindSpecies-response.msg.html

(cl:defclass <FindSpecies-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type reefbot_msgs-msg:SpeciesIDResponse
    :initform (cl:make-instance 'reefbot_msgs-msg:SpeciesIDResponse)))
)

(cl:defclass FindSpecies-response (<FindSpecies-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindSpecies-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindSpecies-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-srv:<FindSpecies-response> is deprecated: use reefbot_msgs-srv:FindSpecies-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <FindSpecies-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-srv:response-val is deprecated.  Use reefbot_msgs-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindSpecies-response>) ostream)
  "Serializes a message object of type '<FindSpecies-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'response) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindSpecies-response>) istream)
  "Deserializes a message object of type '<FindSpecies-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'response) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindSpecies-response>)))
  "Returns string type for a service object of type '<FindSpecies-response>"
  "reefbot_msgs/FindSpeciesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindSpecies-response)))
  "Returns string type for a service object of type 'FindSpecies-response"
  "reefbot_msgs/FindSpeciesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindSpecies-response>)))
  "Returns md5sum for a message object of type '<FindSpecies-response>"
  "7da9366c50b21c9cef266c42d369b77c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindSpecies-response)))
  "Returns md5sum for a message object of type 'FindSpecies-response"
  "7da9366c50b21c9cef266c42d369b77c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindSpecies-response>)))
  "Returns full string definition for message of type '<FindSpecies-response>"
  (cl:format cl:nil "SpeciesIDResponse response~%~%================================================================================~%MSG: reefbot_msgs/SpeciesIDResponse~%# Response to a SpeciesIDRequest that tells the user the best guess of~%# what species were found.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%Header header~%~%# Optional Id to identify the image~%uint64 image_id~%~%# Answers, one for each region in the Request and in the same order.~%SingleSpeciesId[] answers~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: reefbot_msgs/SingleSpeciesId~%# The best few matches for the species in a single region~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# The bounding box where we found the individual~%sensor_msgs/RegionOfInterest bounding_box~%~%# The most likely species in descending order. This array could be~%# empty if there was no good match.~%SpeciesScore[] best_species~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: reefbot_msgs/SpeciesScore~%# The score of a species identification. The score is dependent on the~%# algorithm being used so it doesn't necessarily have semantic meaning~%# except that a higher score is better.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Unique identifier for the species~%uint32 species_id~%~%# Score for the species. Higher is better~%float32 score~%~%# Optional extra information about the score. This will be algorithm~%# dependent and might be useful for debugging~%string meta_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindSpecies-response)))
  "Returns full string definition for message of type 'FindSpecies-response"
  (cl:format cl:nil "SpeciesIDResponse response~%~%================================================================================~%MSG: reefbot_msgs/SpeciesIDResponse~%# Response to a SpeciesIDRequest that tells the user the best guess of~%# what species were found.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%Header header~%~%# Optional Id to identify the image~%uint64 image_id~%~%# Answers, one for each region in the Request and in the same order.~%SingleSpeciesId[] answers~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: reefbot_msgs/SingleSpeciesId~%# The best few matches for the species in a single region~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# The bounding box where we found the individual~%sensor_msgs/RegionOfInterest bounding_box~%~%# The most likely species in descending order. This array could be~%# empty if there was no good match.~%SpeciesScore[] best_species~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: reefbot_msgs/SpeciesScore~%# The score of a species identification. The score is dependent on the~%# algorithm being used so it doesn't necessarily have semantic meaning~%# except that a higher score is better.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: June 2010~%~%# Unique identifier for the species~%uint32 species_id~%~%# Score for the species. Higher is better~%float32 score~%~%# Optional extra information about the score. This will be algorithm~%# dependent and might be useful for debugging~%string meta_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindSpecies-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindSpecies-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FindSpecies-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FindSpecies)))
  'FindSpecies-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FindSpecies)))
  'FindSpecies-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindSpecies)))
  "Returns string type for a service object of type '<FindSpecies>"
  "reefbot_msgs/FindSpecies")