; Auto-generated. Do not edit!


(cl:in-package mbn_msgs-msg)


;//! \htmlinclude MarkersPoses.msg.html

(cl:defclass <MarkersPoses> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (markersPoses
    :reader markersPoses
    :initarg :markersPoses
    :type (cl:vector mbn_msgs-msg:MarkerPose)
   :initform (cl:make-array 0 :element-type 'mbn_msgs-msg:MarkerPose :initial-element (cl:make-instance 'mbn_msgs-msg:MarkerPose))))
)

(cl:defclass MarkersPoses (<MarkersPoses>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MarkersPoses>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MarkersPoses)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbn_msgs-msg:<MarkersPoses> is deprecated: use mbn_msgs-msg:MarkersPoses instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MarkersPoses>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbn_msgs-msg:header-val is deprecated.  Use mbn_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'markersPoses-val :lambda-list '(m))
(cl:defmethod markersPoses-val ((m <MarkersPoses>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbn_msgs-msg:markersPoses-val is deprecated.  Use mbn_msgs-msg:markersPoses instead.")
  (markersPoses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MarkersPoses>) ostream)
  "Serializes a message object of type '<MarkersPoses>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'markersPoses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'markersPoses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MarkersPoses>) istream)
  "Deserializes a message object of type '<MarkersPoses>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'markersPoses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'markersPoses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'mbn_msgs-msg:MarkerPose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MarkersPoses>)))
  "Returns string type for a message object of type '<MarkersPoses>"
  "mbn_msgs/MarkersPoses")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarkersPoses)))
  "Returns string type for a message object of type 'MarkersPoses"
  "mbn_msgs/MarkersPoses")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MarkersPoses>)))
  "Returns md5sum for a message object of type '<MarkersPoses>"
  "85996b685f70d0d47e1dc6988356a1cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MarkersPoses)))
  "Returns md5sum for a message object of type 'MarkersPoses"
  "85996b685f70d0d47e1dc6988356a1cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MarkersPoses>)))
  "Returns full string definition for message of type '<MarkersPoses>"
  (cl:format cl:nil "Header header~%MarkerPose[] markersPoses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: mbn_msgs/MarkerPose~%Header header~%int32 marker_id~%geometry_msgs/Pose poseWRTRobotFrame~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MarkersPoses)))
  "Returns full string definition for message of type 'MarkersPoses"
  (cl:format cl:nil "Header header~%MarkerPose[] markersPoses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: mbn_msgs/MarkerPose~%Header header~%int32 marker_id~%geometry_msgs/Pose poseWRTRobotFrame~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MarkersPoses>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'markersPoses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MarkersPoses>))
  "Converts a ROS message object to a list"
  (cl:list 'MarkersPoses
    (cl:cons ':header (header msg))
    (cl:cons ':markersPoses (markersPoses msg))
))
