; Auto-generated. Do not edit!


(cl:in-package mbn_msgs-msg)


;//! \htmlinclude MarkersIDs.msg.html

(cl:defclass <MarkersIDs> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (markersIDs
    :reader markersIDs
    :initarg :markersIDs
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass MarkersIDs (<MarkersIDs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MarkersIDs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MarkersIDs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbn_msgs-msg:<MarkersIDs> is deprecated: use mbn_msgs-msg:MarkersIDs instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MarkersIDs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbn_msgs-msg:header-val is deprecated.  Use mbn_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'markersIDs-val :lambda-list '(m))
(cl:defmethod markersIDs-val ((m <MarkersIDs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbn_msgs-msg:markersIDs-val is deprecated.  Use mbn_msgs-msg:markersIDs instead.")
  (markersIDs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MarkersIDs>) ostream)
  "Serializes a message object of type '<MarkersIDs>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'markersIDs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'markersIDs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MarkersIDs>) istream)
  "Deserializes a message object of type '<MarkersIDs>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'markersIDs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'markersIDs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MarkersIDs>)))
  "Returns string type for a message object of type '<MarkersIDs>"
  "mbn_msgs/MarkersIDs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarkersIDs)))
  "Returns string type for a message object of type 'MarkersIDs"
  "mbn_msgs/MarkersIDs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MarkersIDs>)))
  "Returns md5sum for a message object of type '<MarkersIDs>"
  "26fde87b102132c5657ff6a5551efbfb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MarkersIDs)))
  "Returns md5sum for a message object of type 'MarkersIDs"
  "26fde87b102132c5657ff6a5551efbfb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MarkersIDs>)))
  "Returns full string definition for message of type '<MarkersIDs>"
  (cl:format cl:nil "Header header~%int32[]  markersIDs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MarkersIDs)))
  "Returns full string definition for message of type 'MarkersIDs"
  (cl:format cl:nil "Header header~%int32[]  markersIDs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MarkersIDs>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'markersIDs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MarkersIDs>))
  "Converts a ROS message object to a list"
  (cl:list 'MarkersIDs
    (cl:cons ':header (header msg))
    (cl:cons ':markersIDs (markersIDs msg))
))
