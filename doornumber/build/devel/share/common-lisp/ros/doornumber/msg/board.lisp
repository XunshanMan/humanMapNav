; Auto-generated. Do not edit!


(cl:in-package doornumber-msg)


;//! \htmlinclude board.msg.html

(cl:defclass <board> (roslisp-msg-protocol:ros-message)
  ((tlx
    :reader tlx
    :initarg :tlx
    :type cl:fixnum
    :initform 0)
   (tly
    :reader tly
    :initarg :tly
    :type cl:fixnum
    :initform 0)
   (brx
    :reader brx
    :initarg :brx
    :type cl:fixnum
    :initform 0)
   (bry
    :reader bry
    :initarg :bry
    :type cl:fixnum
    :initform 0)
   (text
    :reader text
    :initarg :text
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass board (<board>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <board>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'board)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name doornumber-msg:<board> is deprecated: use doornumber-msg:board instead.")))

(cl:ensure-generic-function 'tlx-val :lambda-list '(m))
(cl:defmethod tlx-val ((m <board>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader doornumber-msg:tlx-val is deprecated.  Use doornumber-msg:tlx instead.")
  (tlx m))

(cl:ensure-generic-function 'tly-val :lambda-list '(m))
(cl:defmethod tly-val ((m <board>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader doornumber-msg:tly-val is deprecated.  Use doornumber-msg:tly instead.")
  (tly m))

(cl:ensure-generic-function 'brx-val :lambda-list '(m))
(cl:defmethod brx-val ((m <board>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader doornumber-msg:brx-val is deprecated.  Use doornumber-msg:brx instead.")
  (brx m))

(cl:ensure-generic-function 'bry-val :lambda-list '(m))
(cl:defmethod bry-val ((m <board>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader doornumber-msg:bry-val is deprecated.  Use doornumber-msg:bry instead.")
  (bry m))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <board>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader doornumber-msg:text-val is deprecated.  Use doornumber-msg:text instead.")
  (text m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <board>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader doornumber-msg:confidence-val is deprecated.  Use doornumber-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <board>) ostream)
  "Serializes a message object of type '<board>"
  (cl:let* ((signed (cl:slot-value msg 'tlx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tly)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'brx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bry)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <board>) istream)
  "Deserializes a message object of type '<board>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tlx) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tly) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'brx) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bry) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<board>)))
  "Returns string type for a message object of type '<board>"
  "doornumber/board")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'board)))
  "Returns string type for a message object of type 'board"
  "doornumber/board")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<board>)))
  "Returns md5sum for a message object of type '<board>"
  "41a76418e981dca789597de9cfa6bfa8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'board)))
  "Returns md5sum for a message object of type 'board"
  "41a76418e981dca789597de9cfa6bfa8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<board>)))
  "Returns full string definition for message of type '<board>"
  (cl:format cl:nil "int16 tlx~%int16 tly~%int16 brx~%int16 bry~%string text~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'board)))
  "Returns full string definition for message of type 'board"
  (cl:format cl:nil "int16 tlx~%int16 tly~%int16 brx~%int16 bry~%string text~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <board>))
  (cl:+ 0
     2
     2
     2
     2
     4 (cl:length (cl:slot-value msg 'text))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <board>))
  "Converts a ROS message object to a list"
  (cl:list 'board
    (cl:cons ':tlx (tlx msg))
    (cl:cons ':tly (tly msg))
    (cl:cons ':brx (brx msg))
    (cl:cons ':bry (bry msg))
    (cl:cons ':text (text msg))
    (cl:cons ':confidence (confidence msg))
))
