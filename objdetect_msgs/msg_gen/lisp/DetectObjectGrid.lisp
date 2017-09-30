; Auto-generated. Do not edit!


(cl:in-package objdetect_msgs-msg)


;//! \htmlinclude DetectObjectGrid.msg.html

(cl:defclass <DetectObjectGrid> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (grid
    :reader grid
    :initarg :grid
    :type objdetect_msgs-msg:Grid
    :initform (cl:make-instance 'objdetect_msgs-msg:Grid))
   (mask
    :reader mask
    :initarg :mask
    :type sensor_msgs-msg:MatND
    :initform (cl:make-instance 'sensor_msgs-msg:MatND)))
)

(cl:defclass DetectObjectGrid (<DetectObjectGrid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectObjectGrid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectObjectGrid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objdetect_msgs-msg:<DetectObjectGrid> is deprecated: use objdetect_msgs-msg:DetectObjectGrid instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DetectObjectGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:header-val is deprecated.  Use objdetect_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <DetectObjectGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:image-val is deprecated.  Use objdetect_msgs-msg:image instead.")
  (image m))

(cl:ensure-generic-function 'grid-val :lambda-list '(m))
(cl:defmethod grid-val ((m <DetectObjectGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:grid-val is deprecated.  Use objdetect_msgs-msg:grid instead.")
  (grid m))

(cl:ensure-generic-function 'mask-val :lambda-list '(m))
(cl:defmethod mask-val ((m <DetectObjectGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:mask-val is deprecated.  Use objdetect_msgs-msg:mask instead.")
  (mask m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectObjectGrid>) ostream)
  "Serializes a message object of type '<DetectObjectGrid>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grid) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mask) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectObjectGrid>) istream)
  "Deserializes a message object of type '<DetectObjectGrid>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grid) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mask) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectObjectGrid>)))
  "Returns string type for a message object of type '<DetectObjectGrid>"
  "objdetect_msgs/DetectObjectGrid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectObjectGrid)))
  "Returns string type for a message object of type 'DetectObjectGrid"
  "objdetect_msgs/DetectObjectGrid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectObjectGrid>)))
  "Returns md5sum for a message object of type '<DetectObjectGrid>"
  "b797b0432e7c2ecd0ace33d4c690869f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectObjectGrid)))
  "Returns md5sum for a message object of type 'DetectObjectGrid"
  "b797b0432e7c2ecd0ace33d4c690869f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectObjectGrid>)))
  "Returns full string definition for message of type '<DetectObjectGrid>"
  (cl:format cl:nil "# To detect an object on a w,h,x,y grid. It is more compressed than listing all of the boxes we care about~%~%Header header~%~%# The image to find objects in~%sensor_msgs/Image image~%~%# The (w,h,x,y) grid to search on~%Grid grid~%~%# An optional binary mask that is 4 dimensional (w,h,x,y) and~%# specifies which entries we actually want to search in~%sensor_msgs/MatND mask~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: objdetect_msgs/Grid~%# Specifies a  w,h,x,y dense grid~%# The starting points for the location search~%uint32 minX~%uint32 minY~%~%# The strides in the location space~%uint32 strideX~%uint32 strideY~%~%# The starting points for the scaling~%uint32 minW~%uint32 minH~%~%# The strides in the w, h space. In this case, we step by growing by a~%# fraction, so that width_i is round(minWidth*strideW^i)~%float64 strideW~%float64 strideH~%~%# True if the width and height should be a consistent aspect ratio that are ~%# defined by minW and minH. This reduces the grid to (s,x,y)~%bool fixAspect~%================================================================================~%MSG: sensor_msgs/MatND~%# A message that contains an uncompressed n dimensional~%# matrix. Designed to be compatible with the opencv n-dimensional~%# matrix.~%Header header~%~%int32[] sizes # The size of each dimension in the matrix~%~%string encoding # The data type see src/image_encodings.cpp~%~%bool is_bigendian # Is the data bigendian?~%~%uint8[] data # The actual data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectObjectGrid)))
  "Returns full string definition for message of type 'DetectObjectGrid"
  (cl:format cl:nil "# To detect an object on a w,h,x,y grid. It is more compressed than listing all of the boxes we care about~%~%Header header~%~%# The image to find objects in~%sensor_msgs/Image image~%~%# The (w,h,x,y) grid to search on~%Grid grid~%~%# An optional binary mask that is 4 dimensional (w,h,x,y) and~%# specifies which entries we actually want to search in~%sensor_msgs/MatND mask~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: objdetect_msgs/Grid~%# Specifies a  w,h,x,y dense grid~%# The starting points for the location search~%uint32 minX~%uint32 minY~%~%# The strides in the location space~%uint32 strideX~%uint32 strideY~%~%# The starting points for the scaling~%uint32 minW~%uint32 minH~%~%# The strides in the w, h space. In this case, we step by growing by a~%# fraction, so that width_i is round(minWidth*strideW^i)~%float64 strideW~%float64 strideH~%~%# True if the width and height should be a consistent aspect ratio that are ~%# defined by minW and minH. This reduces the grid to (s,x,y)~%bool fixAspect~%================================================================================~%MSG: sensor_msgs/MatND~%# A message that contains an uncompressed n dimensional~%# matrix. Designed to be compatible with the opencv n-dimensional~%# matrix.~%Header header~%~%int32[] sizes # The size of each dimension in the matrix~%~%string encoding # The data type see src/image_encodings.cpp~%~%bool is_bigendian # Is the data bigendian?~%~%uint8[] data # The actual data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectObjectGrid>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grid))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mask))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectObjectGrid>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectObjectGrid
    (cl:cons ':header (header msg))
    (cl:cons ':image (image msg))
    (cl:cons ':grid (grid msg))
    (cl:cons ':mask (mask msg))
))
