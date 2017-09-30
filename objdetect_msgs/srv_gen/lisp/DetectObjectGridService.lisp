; Auto-generated. Do not edit!


(cl:in-package objdetect_msgs-srv)


;//! \htmlinclude DetectObjectGridService-request.msg.html

(cl:defclass <DetectObjectGridService-request> (roslisp-msg-protocol:ros-message)
  ((request_msg
    :reader request_msg
    :initarg :request_msg
    :type objdetect_msgs-msg:DetectObjectGrid
    :initform (cl:make-instance 'objdetect_msgs-msg:DetectObjectGrid)))
)

(cl:defclass DetectObjectGridService-request (<DetectObjectGridService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectObjectGridService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectObjectGridService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objdetect_msgs-srv:<DetectObjectGridService-request> is deprecated: use objdetect_msgs-srv:DetectObjectGridService-request instead.")))

(cl:ensure-generic-function 'request_msg-val :lambda-list '(m))
(cl:defmethod request_msg-val ((m <DetectObjectGridService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-srv:request_msg-val is deprecated.  Use objdetect_msgs-srv:request_msg instead.")
  (request_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectObjectGridService-request>) ostream)
  "Serializes a message object of type '<DetectObjectGridService-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'request_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectObjectGridService-request>) istream)
  "Deserializes a message object of type '<DetectObjectGridService-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'request_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectObjectGridService-request>)))
  "Returns string type for a service object of type '<DetectObjectGridService-request>"
  "objdetect_msgs/DetectObjectGridServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectObjectGridService-request)))
  "Returns string type for a service object of type 'DetectObjectGridService-request"
  "objdetect_msgs/DetectObjectGridServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectObjectGridService-request>)))
  "Returns md5sum for a message object of type '<DetectObjectGridService-request>"
  "e387077e439fcb0349b2190bd0852bae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectObjectGridService-request)))
  "Returns md5sum for a message object of type 'DetectObjectGridService-request"
  "e387077e439fcb0349b2190bd0852bae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectObjectGridService-request>)))
  "Returns full string definition for message of type '<DetectObjectGridService-request>"
  (cl:format cl:nil "DetectObjectGrid request_msg~%~%================================================================================~%MSG: objdetect_msgs/DetectObjectGrid~%# To detect an object on a w,h,x,y grid. It is more compressed than listing all of the boxes we care about~%~%Header header~%~%# The image to find objects in~%sensor_msgs/Image image~%~%# The (w,h,x,y) grid to search on~%Grid grid~%~%# An optional binary mask that is 4 dimensional (w,h,x,y) and~%# specifies which entries we actually want to search in~%sensor_msgs/MatND mask~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: objdetect_msgs/Grid~%# Specifies a  w,h,x,y dense grid~%# The starting points for the location search~%uint32 minX~%uint32 minY~%~%# The strides in the location space~%uint32 strideX~%uint32 strideY~%~%# The starting points for the scaling~%uint32 minW~%uint32 minH~%~%# The strides in the w, h space. In this case, we step by growing by a~%# fraction, so that width_i is round(minWidth*strideW^i)~%float64 strideW~%float64 strideH~%~%# True if the width and height should be a consistent aspect ratio that are ~%# defined by minW and minH. This reduces the grid to (s,x,y)~%bool fixAspect~%================================================================================~%MSG: sensor_msgs/MatND~%# A message that contains an uncompressed n dimensional~%# matrix. Designed to be compatible with the opencv n-dimensional~%# matrix.~%Header header~%~%int32[] sizes # The size of each dimension in the matrix~%~%string encoding # The data type see src/image_encodings.cpp~%~%bool is_bigendian # Is the data bigendian?~%~%uint8[] data # The actual data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectObjectGridService-request)))
  "Returns full string definition for message of type 'DetectObjectGridService-request"
  (cl:format cl:nil "DetectObjectGrid request_msg~%~%================================================================================~%MSG: objdetect_msgs/DetectObjectGrid~%# To detect an object on a w,h,x,y grid. It is more compressed than listing all of the boxes we care about~%~%Header header~%~%# The image to find objects in~%sensor_msgs/Image image~%~%# The (w,h,x,y) grid to search on~%Grid grid~%~%# An optional binary mask that is 4 dimensional (w,h,x,y) and~%# specifies which entries we actually want to search in~%sensor_msgs/MatND mask~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: objdetect_msgs/Grid~%# Specifies a  w,h,x,y dense grid~%# The starting points for the location search~%uint32 minX~%uint32 minY~%~%# The strides in the location space~%uint32 strideX~%uint32 strideY~%~%# The starting points for the scaling~%uint32 minW~%uint32 minH~%~%# The strides in the w, h space. In this case, we step by growing by a~%# fraction, so that width_i is round(minWidth*strideW^i)~%float64 strideW~%float64 strideH~%~%# True if the width and height should be a consistent aspect ratio that are ~%# defined by minW and minH. This reduces the grid to (s,x,y)~%bool fixAspect~%================================================================================~%MSG: sensor_msgs/MatND~%# A message that contains an uncompressed n dimensional~%# matrix. Designed to be compatible with the opencv n-dimensional~%# matrix.~%Header header~%~%int32[] sizes # The size of each dimension in the matrix~%~%string encoding # The data type see src/image_encodings.cpp~%~%bool is_bigendian # Is the data bigendian?~%~%uint8[] data # The actual data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectObjectGridService-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'request_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectObjectGridService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectObjectGridService-request
    (cl:cons ':request_msg (request_msg msg))
))
;//! \htmlinclude DetectObjectGridService-response.msg.html

(cl:defclass <DetectObjectGridService-response> (roslisp-msg-protocol:ros-message)
  ((scores
    :reader scores
    :initarg :scores
    :type objdetect_msgs-msg:DetectGridScores
    :initform (cl:make-instance 'objdetect_msgs-msg:DetectGridScores)))
)

(cl:defclass DetectObjectGridService-response (<DetectObjectGridService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectObjectGridService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectObjectGridService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objdetect_msgs-srv:<DetectObjectGridService-response> is deprecated: use objdetect_msgs-srv:DetectObjectGridService-response instead.")))

(cl:ensure-generic-function 'scores-val :lambda-list '(m))
(cl:defmethod scores-val ((m <DetectObjectGridService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-srv:scores-val is deprecated.  Use objdetect_msgs-srv:scores instead.")
  (scores m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectObjectGridService-response>) ostream)
  "Serializes a message object of type '<DetectObjectGridService-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'scores) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectObjectGridService-response>) istream)
  "Deserializes a message object of type '<DetectObjectGridService-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'scores) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectObjectGridService-response>)))
  "Returns string type for a service object of type '<DetectObjectGridService-response>"
  "objdetect_msgs/DetectObjectGridServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectObjectGridService-response)))
  "Returns string type for a service object of type 'DetectObjectGridService-response"
  "objdetect_msgs/DetectObjectGridServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectObjectGridService-response>)))
  "Returns md5sum for a message object of type '<DetectObjectGridService-response>"
  "e387077e439fcb0349b2190bd0852bae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectObjectGridService-response)))
  "Returns md5sum for a message object of type 'DetectObjectGridService-response"
  "e387077e439fcb0349b2190bd0852bae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectObjectGridService-response>)))
  "Returns full string definition for message of type '<DetectObjectGridService-response>"
  (cl:format cl:nil "DetectGridScores scores~%~%================================================================================~%MSG: objdetect_msgs/DetectGridScores~%# Specifies socres on a detection grid that runs (x,y,w,h). If the aspect ratio is fixed, this will change to (x,y,s)~%~%Header header~%~%# The (w,h,x,y) grid that has a response~%Grid grid~%~%# A grid of scores across the space that are based on an evaluation~%# for each box.~%sensor_msgs/MatND scores~%~%# An optional binary mask that is 4 dimensional (w,h,x,y) and~%# specifies which entries have valid values~%sensor_msgs/MatND mask~%~%# The processing time to calculate the detection~%std_msgs/Duration processing_time~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: objdetect_msgs/Grid~%# Specifies a  w,h,x,y dense grid~%# The starting points for the location search~%uint32 minX~%uint32 minY~%~%# The strides in the location space~%uint32 strideX~%uint32 strideY~%~%# The starting points for the scaling~%uint32 minW~%uint32 minH~%~%# The strides in the w, h space. In this case, we step by growing by a~%# fraction, so that width_i is round(minWidth*strideW^i)~%float64 strideW~%float64 strideH~%~%# True if the width and height should be a consistent aspect ratio that are ~%# defined by minW and minH. This reduces the grid to (s,x,y)~%bool fixAspect~%================================================================================~%MSG: sensor_msgs/MatND~%# A message that contains an uncompressed n dimensional~%# matrix. Designed to be compatible with the opencv n-dimensional~%# matrix.~%Header header~%~%int32[] sizes # The size of each dimension in the matrix~%~%string encoding # The data type see src/image_encodings.cpp~%~%bool is_bigendian # Is the data bigendian?~%~%uint8[] data # The actual data~%~%================================================================================~%MSG: std_msgs/Duration~%duration data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectObjectGridService-response)))
  "Returns full string definition for message of type 'DetectObjectGridService-response"
  (cl:format cl:nil "DetectGridScores scores~%~%================================================================================~%MSG: objdetect_msgs/DetectGridScores~%# Specifies socres on a detection grid that runs (x,y,w,h). If the aspect ratio is fixed, this will change to (x,y,s)~%~%Header header~%~%# The (w,h,x,y) grid that has a response~%Grid grid~%~%# A grid of scores across the space that are based on an evaluation~%# for each box.~%sensor_msgs/MatND scores~%~%# An optional binary mask that is 4 dimensional (w,h,x,y) and~%# specifies which entries have valid values~%sensor_msgs/MatND mask~%~%# The processing time to calculate the detection~%std_msgs/Duration processing_time~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: objdetect_msgs/Grid~%# Specifies a  w,h,x,y dense grid~%# The starting points for the location search~%uint32 minX~%uint32 minY~%~%# The strides in the location space~%uint32 strideX~%uint32 strideY~%~%# The starting points for the scaling~%uint32 minW~%uint32 minH~%~%# The strides in the w, h space. In this case, we step by growing by a~%# fraction, so that width_i is round(minWidth*strideW^i)~%float64 strideW~%float64 strideH~%~%# True if the width and height should be a consistent aspect ratio that are ~%# defined by minW and minH. This reduces the grid to (s,x,y)~%bool fixAspect~%================================================================================~%MSG: sensor_msgs/MatND~%# A message that contains an uncompressed n dimensional~%# matrix. Designed to be compatible with the opencv n-dimensional~%# matrix.~%Header header~%~%int32[] sizes # The size of each dimension in the matrix~%~%string encoding # The data type see src/image_encodings.cpp~%~%bool is_bigendian # Is the data bigendian?~%~%uint8[] data # The actual data~%~%================================================================================~%MSG: std_msgs/Duration~%duration data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectObjectGridService-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'scores))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectObjectGridService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectObjectGridService-response
    (cl:cons ':scores (scores msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DetectObjectGridService)))
  'DetectObjectGridService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DetectObjectGridService)))
  'DetectObjectGridService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectObjectGridService)))
  "Returns string type for a service object of type '<DetectObjectGridService>"
  "objdetect_msgs/DetectObjectGridService")