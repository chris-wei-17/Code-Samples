; Auto-generated. Do not edit!


(cl:in-package simulator-msg)


;//! \htmlinclude DriveValues.msg.html

(cl:defclass <DriveValues> (roslisp-msg-protocol:ros-message)
  ((pwm_drive
    :reader pwm_drive
    :initarg :pwm_drive
    :type cl:fixnum
    :initform 0)
   (pwm_angle
    :reader pwm_angle
    :initarg :pwm_angle
    :type cl:fixnum
    :initform 0))
)

(cl:defclass DriveValues (<DriveValues>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DriveValues>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DriveValues)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-msg:<DriveValues> is deprecated: use simulator-msg:DriveValues instead.")))

(cl:ensure-generic-function 'pwm_drive-val :lambda-list '(m))
(cl:defmethod pwm_drive-val ((m <DriveValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:pwm_drive-val is deprecated.  Use simulator-msg:pwm_drive instead.")
  (pwm_drive m))

(cl:ensure-generic-function 'pwm_angle-val :lambda-list '(m))
(cl:defmethod pwm_angle-val ((m <DriveValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:pwm_angle-val is deprecated.  Use simulator-msg:pwm_angle instead.")
  (pwm_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DriveValues>) ostream)
  "Serializes a message object of type '<DriveValues>"
  (cl:let* ((signed (cl:slot-value msg 'pwm_drive)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pwm_angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DriveValues>) istream)
  "Deserializes a message object of type '<DriveValues>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwm_drive) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwm_angle) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DriveValues>)))
  "Returns string type for a message object of type '<DriveValues>"
  "simulator/DriveValues")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DriveValues)))
  "Returns string type for a message object of type 'DriveValues"
  "simulator/DriveValues")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DriveValues>)))
  "Returns md5sum for a message object of type '<DriveValues>"
  "180768e2d6cce7b3f71749adb84f7b29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DriveValues)))
  "Returns md5sum for a message object of type 'DriveValues"
  "180768e2d6cce7b3f71749adb84f7b29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DriveValues>)))
  "Returns full string definition for message of type '<DriveValues>"
  (cl:format cl:nil "# PWM duty cycle (0-100%) corresponds to (0-65535) interval~%int16 pwm_drive~%int16 pwm_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DriveValues)))
  "Returns full string definition for message of type 'DriveValues"
  (cl:format cl:nil "# PWM duty cycle (0-100%) corresponds to (0-65535) interval~%int16 pwm_drive~%int16 pwm_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DriveValues>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DriveValues>))
  "Converts a ROS message object to a list"
  (cl:list 'DriveValues
    (cl:cons ':pwm_drive (pwm_drive msg))
    (cl:cons ':pwm_angle (pwm_angle msg))
))
