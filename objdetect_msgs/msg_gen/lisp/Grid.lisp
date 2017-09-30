; Auto-generated. Do not edit!


(cl:in-package objdetect_msgs-msg)


;//! \htmlinclude Grid.msg.html

(cl:defclass <Grid> (roslisp-msg-protocol:ros-message)
  ((minX
    :reader minX
    :initarg :minX
    :type cl:integer
    :initform 0)
   (minY
    :reader minY
    :initarg :minY
    :type cl:integer
    :initform 0)
   (strideX
    :reader strideX
    :initarg :strideX
    :type cl:integer
    :initform 0)
   (strideY
    :reader strideY
    :initarg :strideY
    :type cl:integer
    :initform 0)
   (minW
    :reader minW
    :initarg :minW
    :type cl:integer
    :initform 0)
   (minH
    :reader minH
    :initarg :minH
    :type cl:integer
    :initform 0)
   (strideW
    :reader strideW
    :initarg :strideW
    :type cl:float
    :initform 0.0)
   (strideH
    :reader strideH
    :initarg :strideH
    :type cl:float
    :initform 0.0)
   (fixAspect
    :reader fixAspect
    :initarg :fixAspect
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Grid (<Grid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Grid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Grid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objdetect_msgs-msg:<Grid> is deprecated: use objdetect_msgs-msg:Grid instead.")))

(cl:ensure-generic-function 'minX-val :lambda-list '(m))
(cl:defmethod minX-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:minX-val is deprecated.  Use objdetect_msgs-msg:minX instead.")
  (minX m))

(cl:ensure-generic-function 'minY-val :lambda-list '(m))
(cl:defmethod minY-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:minY-val is deprecated.  Use objdetect_msgs-msg:minY instead.")
  (minY m))

(cl:ensure-generic-function 'strideX-val :lambda-list '(m))
(cl:defmethod strideX-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:strideX-val is deprecated.  Use objdetect_msgs-msg:strideX instead.")
  (strideX m))

(cl:ensure-generic-function 'strideY-val :lambda-list '(m))
(cl:defmethod strideY-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:strideY-val is deprecated.  Use objdetect_msgs-msg:strideY instead.")
  (strideY m))

(cl:ensure-generic-function 'minW-val :lambda-list '(m))
(cl:defmethod minW-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:minW-val is deprecated.  Use objdetect_msgs-msg:minW instead.")
  (minW m))

(cl:ensure-generic-function 'minH-val :lambda-list '(m))
(cl:defmethod minH-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:minH-val is deprecated.  Use objdetect_msgs-msg:minH instead.")
  (minH m))

(cl:ensure-generic-function 'strideW-val :lambda-list '(m))
(cl:defmethod strideW-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:strideW-val is deprecated.  Use objdetect_msgs-msg:strideW instead.")
  (strideW m))

(cl:ensure-generic-function 'strideH-val :lambda-list '(m))
(cl:defmethod strideH-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:strideH-val is deprecated.  Use objdetect_msgs-msg:strideH instead.")
  (strideH m))

(cl:ensure-generic-function 'fixAspect-val :lambda-list '(m))
(cl:defmethod fixAspect-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objdetect_msgs-msg:fixAspect-val is deprecated.  Use objdetect_msgs-msg:fixAspect instead.")
  (fixAspect m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Grid>) ostream)
  "Serializes a message object of type '<Grid>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'minX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'minX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'minX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'minX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'minY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'minY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'minY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'minY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'strideX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'strideX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'strideX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'strideX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'strideY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'strideY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'strideY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'strideY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'minW)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'minW)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'minW)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'minW)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'minH)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'minH)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'minH)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'minH)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'strideW))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'strideH))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fixAspect) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Grid>) istream)
  "Deserializes a message object of type '<Grid>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'minX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'minX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'minX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'minX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'minY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'minY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'minY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'minY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'strideX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'strideX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'strideX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'strideX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'strideY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'strideY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'strideY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'strideY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'minW)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'minW)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'minW)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'minW)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'minH)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'minH)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'minH)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'minH)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'strideW) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'strideH) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'fixAspect) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Grid>)))
  "Returns string type for a message object of type '<Grid>"
  "objdetect_msgs/Grid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Grid)))
  "Returns string type for a message object of type 'Grid"
  "objdetect_msgs/Grid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Grid>)))
  "Returns md5sum for a message object of type '<Grid>"
  "c1403dfd7e97db95b316a1cacfcaecf8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Grid)))
  "Returns md5sum for a message object of type 'Grid"
  "c1403dfd7e97db95b316a1cacfcaecf8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Grid>)))
  "Returns full string definition for message of type '<Grid>"
  (cl:format cl:nil "# Specifies a  w,h,x,y dense grid~%# The starting points for the location search~%uint32 minX~%uint32 minY~%~%# The strides in the location space~%uint32 strideX~%uint32 strideY~%~%# The starting points for the scaling~%uint32 minW~%uint32 minH~%~%# The strides in the w, h space. In this case, we step by growing by a~%# fraction, so that width_i is round(minWidth*strideW^i)~%float64 strideW~%float64 strideH~%~%# True if the width and height should be a consistent aspect ratio that are ~%# defined by minW and minH. This reduces the grid to (s,x,y)~%bool fixAspect~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Grid)))
  "Returns full string definition for message of type 'Grid"
  (cl:format cl:nil "# Specifies a  w,h,x,y dense grid~%# The starting points for the location search~%uint32 minX~%uint32 minY~%~%# The strides in the location space~%uint32 strideX~%uint32 strideY~%~%# The starting points for the scaling~%uint32 minW~%uint32 minH~%~%# The strides in the w, h space. In this case, we step by growing by a~%# fraction, so that width_i is round(minWidth*strideW^i)~%float64 strideW~%float64 strideH~%~%# True if the width and height should be a consistent aspect ratio that are ~%# defined by minW and minH. This reduces the grid to (s,x,y)~%bool fixAspect~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Grid>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Grid>))
  "Converts a ROS message object to a list"
  (cl:list 'Grid
    (cl:cons ':minX (minX msg))
    (cl:cons ':minY (minY msg))
    (cl:cons ':strideX (strideX msg))
    (cl:cons ':strideY (strideY msg))
    (cl:cons ':minW (minW msg))
    (cl:cons ':minH (minH msg))
    (cl:cons ':strideW (strideW msg))
    (cl:cons ':strideH (strideH msg))
    (cl:cons ':fixAspect (fixAspect msg))
))
