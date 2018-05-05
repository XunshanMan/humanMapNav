; Auto-generated. Do not edit!


(cl:in-package doornumber-msg)


;//! \htmlinclude boardArray.msg.html

(cl:defclass <boardArray> (roslisp-msg-protocol:ros-message)
  ((boardArray
    :reader boardArray
    :initarg :boardArray
    :type (cl:vector doornumber-msg:board)
   :initform (cl:make-array 0 :element-type 'doornumber-msg:board :initial-element (cl:make-instance 'doornumber-msg:board))))
)

(cl:defclass boardArray (<boardArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <boardArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'boardArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name doornumber-msg:<boardArray> is deprecated: use doornumber-msg:boardArray instead.")))

(cl:ensure-generic-function 'boardArray-val :lambda-list '(m))
(cl:defmethod boardArray-val ((m <boardArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader doornumber-msg:boardArray-val is deprecated.  Use doornumber-msg:boardArray instead.")
  (boardArray m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <boardArray>) ostream)
  "Serializes a message object of type '<boardArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'boardArray))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'boardArray))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <boardArray>) istream)
  "Deserializes a message object of type '<boardArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'boardArray) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'boardArray)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'doornumber-msg:board))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<boardArray>)))
  "Returns string type for a message object of type '<boardArray>"
  "doornumber/boardArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'boardArray)))
  "Returns string type for a message object of type 'boardArray"
  "doornumber/boardArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<boardArray>)))
  "Returns md5sum for a message object of type '<boardArray>"
  "67a68e0f54af64c049c8d5824d305a60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'boardArray)))
  "Returns md5sum for a message object of type 'boardArray"
  "67a68e0f54af64c049c8d5824d305a60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<boardArray>)))
  "Returns full string definition for message of type '<boardArray>"
  (cl:format cl:nil "board[] boardArray~%~%================================================================================~%MSG: doornumber/board~%int16 tlx~%int16 tly~%int16 brx~%int16 bry~%string text~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'boardArray)))
  "Returns full string definition for message of type 'boardArray"
  (cl:format cl:nil "board[] boardArray~%~%================================================================================~%MSG: doornumber/board~%int16 tlx~%int16 tly~%int16 brx~%int16 bry~%string text~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <boardArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'boardArray) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <boardArray>))
  "Converts a ROS message object to a list"
  (cl:list 'boardArray
    (cl:cons ':boardArray (boardArray msg))
))
