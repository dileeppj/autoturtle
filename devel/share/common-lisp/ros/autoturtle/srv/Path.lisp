; Auto-generated. Do not edit!


(cl:in-package autoturtle-srv)


;//! \htmlinclude Path-request.msg.html

(cl:defclass <Path-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (stop
    :reader stop
    :initarg :stop
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Path-request (<Path-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Path-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Path-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autoturtle-srv:<Path-request> is deprecated: use autoturtle-srv:Path-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <Path-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autoturtle-srv:start-val is deprecated.  Use autoturtle-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'stop-val :lambda-list '(m))
(cl:defmethod stop-val ((m <Path-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autoturtle-srv:stop-val is deprecated.  Use autoturtle-srv:stop instead.")
  (stop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Path-request>) ostream)
  "Serializes a message object of type '<Path-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'start))))
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
   (cl:slot-value msg 'start))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'stop))))
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
   (cl:slot-value msg 'stop))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Path-request>) istream)
  "Deserializes a message object of type '<Path-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'start) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'start)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'stop) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'stop)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Path-request>)))
  "Returns string type for a service object of type '<Path-request>"
  "autoturtle/PathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Path-request)))
  "Returns string type for a service object of type 'Path-request"
  "autoturtle/PathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Path-request>)))
  "Returns md5sum for a message object of type '<Path-request>"
  "d32b3108e0653dfb8af9c9692bc47b31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Path-request)))
  "Returns md5sum for a message object of type 'Path-request"
  "d32b3108e0653dfb8af9c9692bc47b31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Path-request>)))
  "Returns full string definition for message of type '<Path-request>"
  (cl:format cl:nil "int32[] start~%int32[] stop~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Path-request)))
  "Returns full string definition for message of type 'Path-request"
  (cl:format cl:nil "int32[] start~%int32[] stop~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Path-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'start) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'stop) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Path-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Path-request
    (cl:cons ':start (start msg))
    (cl:cons ':stop (stop msg))
))
;//! \htmlinclude Path-response.msg.html

(cl:defclass <Path-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type cl:string
    :initform ""))
)

(cl:defclass Path-response (<Path-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Path-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Path-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autoturtle-srv:<Path-response> is deprecated: use autoturtle-srv:Path-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <Path-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autoturtle-srv:path-val is deprecated.  Use autoturtle-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Path-response>) ostream)
  "Serializes a message object of type '<Path-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Path-response>) istream)
  "Deserializes a message object of type '<Path-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Path-response>)))
  "Returns string type for a service object of type '<Path-response>"
  "autoturtle/PathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Path-response)))
  "Returns string type for a service object of type 'Path-response"
  "autoturtle/PathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Path-response>)))
  "Returns md5sum for a message object of type '<Path-response>"
  "d32b3108e0653dfb8af9c9692bc47b31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Path-response)))
  "Returns md5sum for a message object of type 'Path-response"
  "d32b3108e0653dfb8af9c9692bc47b31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Path-response>)))
  "Returns full string definition for message of type '<Path-response>"
  (cl:format cl:nil "string path~%# int32[] path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Path-response)))
  "Returns full string definition for message of type 'Path-response"
  (cl:format cl:nil "string path~%# int32[] path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Path-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Path-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Path-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Path)))
  'Path-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Path)))
  'Path-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Path)))
  "Returns string type for a service object of type '<Path>"
  "autoturtle/Path")