; Auto-generated. Do not edit!


(cl:in-package Frank_control-srv)


;//! \htmlinclude GetHandAngles-request.msg.html

(cl:defclass <GetHandAngles-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetHandAngles-request (<GetHandAngles-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHandAngles-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHandAngles-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Frank_control-srv:<GetHandAngles-request> is deprecated: use Frank_control-srv:GetHandAngles-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHandAngles-request>) ostream)
  "Serializes a message object of type '<GetHandAngles-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHandAngles-request>) istream)
  "Deserializes a message object of type '<GetHandAngles-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHandAngles-request>)))
  "Returns string type for a service object of type '<GetHandAngles-request>"
  "Frank_control/GetHandAnglesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHandAngles-request)))
  "Returns string type for a service object of type 'GetHandAngles-request"
  "Frank_control/GetHandAnglesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHandAngles-request>)))
  "Returns md5sum for a message object of type '<GetHandAngles-request>"
  "ae6e268c4bfe9632c21995915fa1b5ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHandAngles-request)))
  "Returns md5sum for a message object of type 'GetHandAngles-request"
  "ae6e268c4bfe9632c21995915fa1b5ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHandAngles-request>)))
  "Returns full string definition for message of type '<GetHandAngles-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHandAngles-request)))
  "Returns full string definition for message of type 'GetHandAngles-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHandAngles-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHandAngles-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHandAngles-request
))
;//! \htmlinclude GetHandAngles-response.msg.html

(cl:defclass <GetHandAngles-response> (roslisp-msg-protocol:ros-message)
  ((thumb_bend
    :reader thumb_bend
    :initarg :thumb_bend
    :type cl:float
    :initform 0.0)
   (index_bend
    :reader index_bend
    :initarg :index_bend
    :type cl:float
    :initform 0.0)
   (middle_bend
    :reader middle_bend
    :initarg :middle_bend
    :type cl:float
    :initform 0.0)
   (ring_bend
    :reader ring_bend
    :initarg :ring_bend
    :type cl:float
    :initform 0.0)
   (pinky_bend
    :reader pinky_bend
    :initarg :pinky_bend
    :type cl:float
    :initform 0.0)
   (thumb_rot
    :reader thumb_rot
    :initarg :thumb_rot
    :type cl:float
    :initform 0.0)
   (detected
    :reader detected
    :initarg :detected
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetHandAngles-response (<GetHandAngles-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHandAngles-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHandAngles-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Frank_control-srv:<GetHandAngles-response> is deprecated: use Frank_control-srv:GetHandAngles-response instead.")))

(cl:ensure-generic-function 'thumb_bend-val :lambda-list '(m))
(cl:defmethod thumb_bend-val ((m <GetHandAngles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Frank_control-srv:thumb_bend-val is deprecated.  Use Frank_control-srv:thumb_bend instead.")
  (thumb_bend m))

(cl:ensure-generic-function 'index_bend-val :lambda-list '(m))
(cl:defmethod index_bend-val ((m <GetHandAngles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Frank_control-srv:index_bend-val is deprecated.  Use Frank_control-srv:index_bend instead.")
  (index_bend m))

(cl:ensure-generic-function 'middle_bend-val :lambda-list '(m))
(cl:defmethod middle_bend-val ((m <GetHandAngles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Frank_control-srv:middle_bend-val is deprecated.  Use Frank_control-srv:middle_bend instead.")
  (middle_bend m))

(cl:ensure-generic-function 'ring_bend-val :lambda-list '(m))
(cl:defmethod ring_bend-val ((m <GetHandAngles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Frank_control-srv:ring_bend-val is deprecated.  Use Frank_control-srv:ring_bend instead.")
  (ring_bend m))

(cl:ensure-generic-function 'pinky_bend-val :lambda-list '(m))
(cl:defmethod pinky_bend-val ((m <GetHandAngles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Frank_control-srv:pinky_bend-val is deprecated.  Use Frank_control-srv:pinky_bend instead.")
  (pinky_bend m))

(cl:ensure-generic-function 'thumb_rot-val :lambda-list '(m))
(cl:defmethod thumb_rot-val ((m <GetHandAngles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Frank_control-srv:thumb_rot-val is deprecated.  Use Frank_control-srv:thumb_rot instead.")
  (thumb_rot m))

(cl:ensure-generic-function 'detected-val :lambda-list '(m))
(cl:defmethod detected-val ((m <GetHandAngles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Frank_control-srv:detected-val is deprecated.  Use Frank_control-srv:detected instead.")
  (detected m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHandAngles-response>) ostream)
  "Serializes a message object of type '<GetHandAngles-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thumb_bend))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'index_bend))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'middle_bend))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ring_bend))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pinky_bend))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thumb_rot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'detected) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHandAngles-response>) istream)
  "Deserializes a message object of type '<GetHandAngles-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thumb_bend) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'index_bend) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'middle_bend) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ring_bend) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pinky_bend) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thumb_rot) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'detected) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHandAngles-response>)))
  "Returns string type for a service object of type '<GetHandAngles-response>"
  "Frank_control/GetHandAnglesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHandAngles-response)))
  "Returns string type for a service object of type 'GetHandAngles-response"
  "Frank_control/GetHandAnglesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHandAngles-response>)))
  "Returns md5sum for a message object of type '<GetHandAngles-response>"
  "ae6e268c4bfe9632c21995915fa1b5ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHandAngles-response)))
  "Returns md5sum for a message object of type 'GetHandAngles-response"
  "ae6e268c4bfe9632c21995915fa1b5ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHandAngles-response>)))
  "Returns full string definition for message of type '<GetHandAngles-response>"
  (cl:format cl:nil "float32 thumb_bend~%float32 index_bend~%float32 middle_bend~%float32 ring_bend~%float32 pinky_bend~%float32 thumb_rot~%bool detected ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHandAngles-response)))
  "Returns full string definition for message of type 'GetHandAngles-response"
  (cl:format cl:nil "float32 thumb_bend~%float32 index_bend~%float32 middle_bend~%float32 ring_bend~%float32 pinky_bend~%float32 thumb_rot~%bool detected ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHandAngles-response>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHandAngles-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHandAngles-response
    (cl:cons ':thumb_bend (thumb_bend msg))
    (cl:cons ':index_bend (index_bend msg))
    (cl:cons ':middle_bend (middle_bend msg))
    (cl:cons ':ring_bend (ring_bend msg))
    (cl:cons ':pinky_bend (pinky_bend msg))
    (cl:cons ':thumb_rot (thumb_rot msg))
    (cl:cons ':detected (detected msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetHandAngles)))
  'GetHandAngles-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetHandAngles)))
  'GetHandAngles-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHandAngles)))
  "Returns string type for a service object of type '<GetHandAngles>"
  "Frank_control/GetHandAngles")