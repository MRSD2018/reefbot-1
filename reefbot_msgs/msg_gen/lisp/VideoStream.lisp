; Auto-generated. Do not edit!


(cl:in-package reefbot_msgs-msg)


;//! \htmlinclude VideoStream.msg.html

(cl:defclass <VideoStream> (roslisp-msg-protocol:ros-message)
  ((url
    :reader url
    :initarg :url
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass VideoStream (<VideoStream>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VideoStream>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VideoStream)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reefbot_msgs-msg:<VideoStream> is deprecated: use reefbot_msgs-msg:VideoStream instead.")))

(cl:ensure-generic-function 'url-val :lambda-list '(m))
(cl:defmethod url-val ((m <VideoStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reefbot_msgs-msg:url-val is deprecated.  Use reefbot_msgs-msg:url instead.")
  (url m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VideoStream>) ostream)
  "Serializes a message object of type '<VideoStream>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'url) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VideoStream>) istream)
  "Deserializes a message object of type '<VideoStream>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'url) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VideoStream>)))
  "Returns string type for a message object of type '<VideoStream>"
  "reefbot_msgs/VideoStream")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VideoStream)))
  "Returns string type for a message object of type 'VideoStream"
  "reefbot_msgs/VideoStream")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VideoStream>)))
  "Returns md5sum for a message object of type '<VideoStream>"
  "cd586f51ec0610ad4479871c38166fc8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VideoStream)))
  "Returns md5sum for a message object of type 'VideoStream"
  "cd586f51ec0610ad4479871c38166fc8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VideoStream>)))
  "Returns full string definition for message of type '<VideoStream>"
  (cl:format cl:nil "# This message specifies the url that the video stream is being~%# broadcast at. This allows us to change the video settings on the fly~%# by sending this message.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: July 2010~%~%std_msgs/String url~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VideoStream)))
  "Returns full string definition for message of type 'VideoStream"
  (cl:format cl:nil "# This message specifies the url that the video stream is being~%# broadcast at. This allows us to change the video settings on the fly~%# by sending this message.~%#~%# Author: Mark Desnoyer (markd@cmu.edu)~%# Date: July 2010~%~%std_msgs/String url~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VideoStream>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'url))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VideoStream>))
  "Converts a ROS message object to a list"
  (cl:list 'VideoStream
    (cl:cons ':url (url msg))
))
