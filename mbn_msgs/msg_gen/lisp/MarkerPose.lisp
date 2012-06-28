; Auto-generated. Do not edit!


(cl:in-package mbn_msgs-msg)


;//! \htmlinclude MarkerPose.msg.html

(cl:defclass <MarkerPose> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (marker_id
    :reader marker_id
    :initarg :marker_id
    :type cl:integer
    :initform 0)
   (poseWRTRobotFrame
    :reader poseWRTRobotFrame
    :initarg :poseWRTRobotFrame
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass MarkerPose (<MarkerPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MarkerPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MarkerPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbn_msgs-msg:<MarkerPose> is deprecated: use mbn_msgs-msg:MarkerPose instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MarkerPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbn_msgs-msg:header-val is deprecated.  Use mbn_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'marker_id-val :lambda-list '(m))
(cl:defmethod marker_id-val ((m <MarkerPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbn_msgs-msg:marker_id-val is deprecated.  Use mbn_msgs-msg:marker_id instead.")
  (marker_id m))

(cl:ensure-generic-function 'poseWRTRobotFrame-val :lambda-list '(m))
(cl:defmethod poseWRTRobotFrame-val ((m <MarkerPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbn_msgs-msg:poseWRTRobotFrame-val is deprecated.  Use mbn_msgs-msg:poseWRTRobotFrame instead.")
  (poseWRTRobotFrame m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MarkerPose>) ostream)
  "Serializes a message object of type '<MarkerPose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'marker_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'poseWRTRobotFrame) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MarkerPose>) istream)
  "Deserializes a message object of type '<MarkerPose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marker_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'poseWRTRobotFrame) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MarkerPose>)))
  "Returns string type for a message object of type '<MarkerPose>"
  "mbn_msgs/MarkerPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarkerPose)))
  "Returns string type for a message object of type 'MarkerPose"
  "mbn_msgs/MarkerPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MarkerPose>)))
  "Returns md5sum for a message object of type '<MarkerPose>"
  "6f04648c896b99690375cf7c51d36eb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MarkerPose)))
  "Returns md5sum for a message object of type 'MarkerPose"
  "6f04648c896b99690375cf7c51d36eb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MarkerPose>)))
  "Returns full string definition for message of type '<MarkerPose>"
  (cl:format cl:nil "Header header~%int32 marker_id~%geometry_msgs/Pose poseWRTRobotFrame~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MarkerPose)))
  "Returns full string definition for message of type 'MarkerPose"
  (cl:format cl:nil "Header header~%int32 marker_id~%geometry_msgs/Pose poseWRTRobotFrame~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MarkerPose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'poseWRTRobotFrame))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MarkerPose>))
  "Converts a ROS message object to a list"
  (cl:list 'MarkerPose
    (cl:cons ':header (header msg))
    (cl:cons ':marker_id (marker_id msg))
    (cl:cons ':poseWRTRobotFrame (poseWRTRobotFrame msg))
))
