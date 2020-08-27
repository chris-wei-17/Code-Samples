; Auto-generated. Do not edit!


(cl:in-package simulator-msg)


;//! \htmlinclude CarControlData.msg.html

(cl:defclass <CarControlData> (roslisp-msg-protocol:ros-message)
  ((cv_speed
    :reader cv_speed
    :initarg :cv_speed
    :type cl:float
    :initform 0.0)
   (cv_turn_zero
    :reader cv_turn_zero
    :initarg :cv_turn_zero
    :type cl:float
    :initform 0.0)
   (cv_turn_speed
    :reader cv_turn_speed
    :initarg :cv_turn_speed
    :type cl:float
    :initform 0.0)
   (vp_speed
    :reader vp_speed
    :initarg :vp_speed
    :type cl:fixnum
    :initform 0)
   (vp_turn_zero
    :reader vp_turn_zero
    :initarg :vp_turn_zero
    :type cl:fixnum
    :initform 0)
   (vp_turn_speed
    :reader vp_turn_speed
    :initarg :vp_turn_speed
    :type cl:fixnum
    :initform 0)
   (vesc_speed
    :reader vesc_speed
    :initarg :vesc_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass CarControlData (<CarControlData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CarControlData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CarControlData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-msg:<CarControlData> is deprecated: use simulator-msg:CarControlData instead.")))

(cl:ensure-generic-function 'cv_speed-val :lambda-list '(m))
(cl:defmethod cv_speed-val ((m <CarControlData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:cv_speed-val is deprecated.  Use simulator-msg:cv_speed instead.")
  (cv_speed m))

(cl:ensure-generic-function 'cv_turn_zero-val :lambda-list '(m))
(cl:defmethod cv_turn_zero-val ((m <CarControlData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:cv_turn_zero-val is deprecated.  Use simulator-msg:cv_turn_zero instead.")
  (cv_turn_zero m))

(cl:ensure-generic-function 'cv_turn_speed-val :lambda-list '(m))
(cl:defmethod cv_turn_speed-val ((m <CarControlData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:cv_turn_speed-val is deprecated.  Use simulator-msg:cv_turn_speed instead.")
  (cv_turn_speed m))

(cl:ensure-generic-function 'vp_speed-val :lambda-list '(m))
(cl:defmethod vp_speed-val ((m <CarControlData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:vp_speed-val is deprecated.  Use simulator-msg:vp_speed instead.")
  (vp_speed m))

(cl:ensure-generic-function 'vp_turn_zero-val :lambda-list '(m))
(cl:defmethod vp_turn_zero-val ((m <CarControlData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:vp_turn_zero-val is deprecated.  Use simulator-msg:vp_turn_zero instead.")
  (vp_turn_zero m))

(cl:ensure-generic-function 'vp_turn_speed-val :lambda-list '(m))
(cl:defmethod vp_turn_speed-val ((m <CarControlData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:vp_turn_speed-val is deprecated.  Use simulator-msg:vp_turn_speed instead.")
  (vp_turn_speed m))

(cl:ensure-generic-function 'vesc_speed-val :lambda-list '(m))
(cl:defmethod vesc_speed-val ((m <CarControlData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:vesc_speed-val is deprecated.  Use simulator-msg:vesc_speed instead.")
  (vesc_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CarControlData>) ostream)
  "Serializes a message object of type '<CarControlData>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'cv_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'cv_turn_zero))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'cv_turn_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'vp_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'vp_turn_zero)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'vp_turn_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vesc_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CarControlData>) istream)
  "Deserializes a message object of type '<CarControlData>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cv_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cv_turn_zero) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cv_turn_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vp_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vp_turn_zero) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vp_turn_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vesc_speed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CarControlData>)))
  "Returns string type for a message object of type '<CarControlData>"
  "simulator/CarControlData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CarControlData)))
  "Returns string type for a message object of type 'CarControlData"
  "simulator/CarControlData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CarControlData>)))
  "Returns md5sum for a message object of type '<CarControlData>"
  "324d1d835c12674bf092534c6d2533e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CarControlData)))
  "Returns md5sum for a message object of type 'CarControlData"
  "324d1d835c12674bf092534c6d2533e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CarControlData>)))
  "Returns full string definition for message of type '<CarControlData>"
  (cl:format cl:nil "# Constants from controlling the car via node~%~%# cmd_vel~%float64 cv_speed~%~%float64 cv_turn_zero~%float64 cv_turn_speed~%~%# drive_pwm~%int16 vp_speed~%~%int16 vp_turn_zero~%int16 vp_turn_speed~%~%# VESC~%float64 vesc_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CarControlData)))
  "Returns full string definition for message of type 'CarControlData"
  (cl:format cl:nil "# Constants from controlling the car via node~%~%# cmd_vel~%float64 cv_speed~%~%float64 cv_turn_zero~%float64 cv_turn_speed~%~%# drive_pwm~%int16 vp_speed~%~%int16 vp_turn_zero~%int16 vp_turn_speed~%~%# VESC~%float64 vesc_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CarControlData>))
  (cl:+ 0
     8
     8
     8
     2
     2
     2
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CarControlData>))
  "Converts a ROS message object to a list"
  (cl:list 'CarControlData
    (cl:cons ':cv_speed (cv_speed msg))
    (cl:cons ':cv_turn_zero (cv_turn_zero msg))
    (cl:cons ':cv_turn_speed (cv_turn_speed msg))
    (cl:cons ':vp_speed (vp_speed msg))
    (cl:cons ':vp_turn_zero (vp_turn_zero msg))
    (cl:cons ':vp_turn_speed (vp_turn_speed msg))
    (cl:cons ':vesc_speed (vesc_speed msg))
))
