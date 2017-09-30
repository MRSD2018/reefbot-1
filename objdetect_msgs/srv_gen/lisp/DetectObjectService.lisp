; Auto-generated. Do not edit!


(cl:in-package objdetect_msgs-srv)


;//! \htmlinclude DetectObjectService-request.msg.html

(cl:defclass <DetectObjectService-request> (roslisp-msg-protocol:ros-message)
  ((request_msg
    :reader request_msg
    :initarg :request_msg
    :type objdetect_msgs-msg:DetectObject
    :initform (cl:make-instance 'objdetect_msgs-msg:DetectObject)))
)

(cl:defclass DetectObjectService-request (<DetectObjectService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectObjectService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectObjectService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objdetect_msgs-srv:<DetectObjectService-request> is deprecated: use objdetect_msgs-srv:DetectObjectService-request instead.")))

(cl:ensure-generic-function 'request_msg-val :lambda-list '(m))
(cl:defmethod request_msg-val ((m <DetectObjectService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-srv:request_msg-val is deprecated.  Use objdetect_msgs-srv:request_msg instead.")
  (request_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectObjectService-request>) ostream)
  "Serializes a message object of type '<DetectObjectService-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'request_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectObjectService-request>) istream)
  "Deserializes a message object of type '<DetectObjectService-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'request_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectObjectService-request>)))
  "Returns string type for a service object of type '<DetectObjectService-request>"
  "objdetect_msgs/DetectObjectServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectObjectService-request)))
  "Returns string type for a service object of type 'DetectObjectService-request"
  "objdetect_msgs/DetectObjectServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectObjectService-request>)))
  "Returns md5sum for a message object of type '<DetectObjectService-request>"
  "1ed709a3e7114464833850e26e7d3057")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectObjectService-request)))
  "Returns md5sum for a message object of type 'DetectObjectService-request"
  "1ed709a3e7114464833850e26e7d3057")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectObjectService-request>)))
  "Returns full string definition for message of type '<DetectObjectService-request>"
  (cl:format cl:nil "DetectObject request_msg~%~%================================================================================~%MSG: objdetect_msgs/DetectObject~%Header header~%~%# The image to find objects in~%sensor_msgs/Image image~%~%# Regions of interest to look for the object. If it is empty, search~%# through the entire image~%sensor_msgs/RegionOfInterest[] regions~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectObjectService-request)))
  "Returns full string definition for message of type 'DetectObjectService-request"
  (cl:format cl:nil "DetectObject request_msg~%~%================================================================================~%MSG: objdetect_msgs/DetectObject~%Header header~%~%# The image to find objects in~%sensor_msgs/Image image~%~%# Regions of interest to look for the object. If it is empty, search~%# through the entire image~%sensor_msgs/RegionOfInterest[] regions~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectObjectService-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'request_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectObjectService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectObjectService-request
    (cl:cons ':request_msg (request_msg msg))
))
;//! \htmlinclude DetectObjectService-response.msg.html

(cl:defclass <DetectObjectService-response> (roslisp-msg-protocol:ros-message)
  ((detections
    :reader detections
    :initarg :detections
    :type objdetect_msgs-msg:DetectionArray
    :initform (cl:make-instance 'objdetect_msgs-msg:DetectionArray))
   (processing_time
    :reader processing_time
    :initarg :processing_time
    :type std_msgs-msg:Duration
    :initform (cl:make-instance 'std_msgs-msg:Duration)))
)

(cl:defclass DetectObjectService-response (<DetectObjectService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectObjectService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectObjectService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objdetect_msgs-srv:<DetectObjectService-response> is deprecated: use objdetect_msgs-srv:DetectObjectService-response instead.")))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <DetectObjectService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-srv:detections-val is deprecated.  Use objdetect_msgs-srv:detections instead.")
  (detections m))

(cl:ensure-generic-function 'processing_time-val :lambda-list '(m))
(cl:defmethod processing_time-val ((m <DetectObjectService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-srv:processing_time-val is deprecated.  Use objdetect_msgs-srv:processing_time instead.")
  (processing_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectObjectService-response>) ostream)
  "Serializes a message object of type '<DetectObjectService-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'detections) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'processing_time) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectObjectService-response>) istream)
  "Deserializes a message object of type '<DetectObjectService-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'detections) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'processing_time) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectObjectService-response>)))
  "Returns string type for a service object of type '<DetectObjectService-response>"
  "objdetect_msgs/DetectObjectServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectObjectService-response)))
  "Returns string type for a service object of type 'DetectObjectService-response"
  "objdetect_msgs/DetectObjectServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectObjectService-response>)))
  "Returns md5sum for a message object of type '<DetectObjectService-response>"
  "1ed709a3e7114464833850e26e7d3057")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectObjectService-response)))
  "Returns md5sum for a message object of type 'DetectObjectService-response"
  "1ed709a3e7114464833850e26e7d3057")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectObjectService-response>)))
  "Returns full string definition for message of type '<DetectObjectService-response>"
  (cl:format cl:nil "DetectionArray detections~%std_msgs/Duration processing_time~%~%================================================================================~%MSG: objdetect_msgs/DetectionArray~%Header header~%Detection[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: objdetect_msgs/Detection~%# A single detection from the object detector~%Header header~%~%# Optional label for the detection~%string label~%~%# Name of the detector used~%string detector~%~%# Quality of the detection~%float32 score~%~%# Mask specifying the location of the detection~%Mask mask~%================================================================================~%MSG: objdetect_msgs/Mask~%# this message is used to mark where an object is present in an image~%# this can be done either by a roi region on the image (less precise)~%# or a mask (more precise)~%~%sensor_msgs/RegionOfInterest roi~%~%# in the case when mask is used, 'roi' specifies the image region and 'mask'~%# (which should be of the same size) a binary mask in that region~%sensor_msgs/Image mask~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Duration~%duration data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectObjectService-response)))
  "Returns full string definition for message of type 'DetectObjectService-response"
  (cl:format cl:nil "DetectionArray detections~%std_msgs/Duration processing_time~%~%================================================================================~%MSG: objdetect_msgs/DetectionArray~%Header header~%Detection[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: objdetect_msgs/Detection~%# A single detection from the object detector~%Header header~%~%# Optional label for the detection~%string label~%~%# Name of the detector used~%string detector~%~%# Quality of the detection~%float32 score~%~%# Mask specifying the location of the detection~%Mask mask~%================================================================================~%MSG: objdetect_msgs/Mask~%# this message is used to mark where an object is present in an image~%# this can be done either by a roi region on the image (less precise)~%# or a mask (more precise)~%~%sensor_msgs/RegionOfInterest roi~%~%# in the case when mask is used, 'roi' specifies the image region and 'mask'~%# (which should be of the same size) a binary mask in that region~%sensor_msgs/Image mask~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Duration~%duration data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectObjectService-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'detections))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'processing_time))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectObjectService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectObjectService-response
    (cl:cons ':detections (detections msg))
    (cl:cons ':processing_time (processing_time msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DetectObjectService)))
  'DetectObjectService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DetectObjectService)))
  'DetectObjectService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectObjectService)))
  "Returns string type for a service object of type '<DetectObjectService>"
  "objdetect_msgs/DetectObjectService")