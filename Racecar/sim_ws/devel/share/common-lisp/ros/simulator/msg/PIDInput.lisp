; Auto-generated. Do not edit!


(cl:in-package simulator-msg)


;//! \htmlinclude PIDInput.msg.html

(cl:defclass <PIDInput> (roslisp-msg-protocol:ros-message)
  ((pid_vel
    :reader pid_vel
    :initarg :pid_vel
    :type cl:float
    :initform 0.0)
   (pid_error
    :reader pid_error
    :initarg :pid_error
    :type cl:float
    :initform 0.0))
)

(cl:defclass PIDInput (<PIDInput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PIDInput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PIDInput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-msg:<PIDInput> is deprecated: use simulator-msg:PIDInput instead.")))

(cl:ensure-generic-function 'pid_vel-val :lambda-list '(m))
(cl:defmethod pid_vel-val ((m <PIDInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:pid_vel-val is deprecated.  Use simulator-msg:pid_vel instead.")
  (pid_vel m))

(cl:ensure-generic-function 'pid_error-val :lambda-list '(m))
(cl:defmethod pid_error-val ((m <PIDInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:pid_error-val is deprecated.  Use simulator-msg:pid_error instead.")
  (pid_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PIDInput>) ostream)
  "Serializes a message object of type '<PIDInput>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pid_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pid_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PIDInput>) istream)
  "Deserializes a message object of type '<PIDInput>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pid_vel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pid_error) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PIDInput>)))
  "Returns string type for a message object of type '<PIDInput>"
  "simulator/PIDInput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PIDInput)))
  "Returns string type for a message object of type 'PIDInput"
  "simulator/PIDInput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PIDInput>)))
  "Returns md5sum for a message object of type '<PIDInput>"
  "15d51ace2dba29e1b19e1332c9d46c17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PIDInput)))
  "Returns md5sum for a message object of type 'PIDInput"
  "15d51ace2dba29e1b19e1332c9d46c17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PIDInput>)))
  "Returns full string definition for message of type '<PIDInput>"
  (cl:format cl:nil "float32 pid_vel~%float32 pid_error~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PIDInput)))
  "Returns full string definition for message of type 'PIDInput"
  (cl:format cl:nil "float32 pid_vel~%float32 pid_error~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PIDInput>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PIDInput>))
  "Converts a ROS message object to a list"
  (cl:list 'PIDInput
    (cl:cons ':pid_vel (pid_vel msg))
    (cl:cons ':pid_error (pid_error msg))
))
