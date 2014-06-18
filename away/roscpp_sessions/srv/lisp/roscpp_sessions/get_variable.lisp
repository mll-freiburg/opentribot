; Auto-generated. Do not edit!


(in-package roscpp_sessions-srv)


;//! \htmlinclude get_variable-request.msg.html

(defclass <get_variable-request> (ros-message)
  ((variable
    :reader variable-val
    :initarg :variable
    :type string
    :initform ""))
)
(defmethod serialize ((msg <get_variable-request>) ostream)
  "Serializes a message object of type '<get_variable-request>"
  (let ((__ros_str_len (length (slot-value msg 'variable))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'variable))
)
(defmethod deserialize ((msg <get_variable-request>) istream)
  "Deserializes a message object of type '<get_variable-request>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'variable) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'variable) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<get_variable-request>)))
  "Returns string type for a service object of type '<get_variable-request>"
  "roscpp_sessions/get_variableRequest")
(defmethod md5sum ((type (eql '<get_variable-request>)))
  "Returns md5sum for a message object of type '<get_variable-request>"
  "e366705baa8c302f52752de97161f798")
(defmethod message-definition ((type (eql '<get_variable-request>)))
  "Returns full string definition for message of type '<get_variable-request>"
  (format nil "string variable~%~%"))
(defmethod serialization-length ((msg <get_variable-request>))
  (+ 0
     4 (length (slot-value msg 'variable))
))
(defmethod ros-message-to-list ((msg <get_variable-request>))
  "Converts a ROS message object to a list"
  (list '<get_variable-request>
    (cons ':variable (variable-val msg))
))
;//! \htmlinclude get_variable-response.msg.html

(defclass <get_variable-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <get_variable-response>) ostream)
  "Serializes a message object of type '<get_variable-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'result)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'result)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <get_variable-response>) istream)
  "Deserializes a message object of type '<get_variable-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'result)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'result)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<get_variable-response>)))
  "Returns string type for a service object of type '<get_variable-response>"
  "roscpp_sessions/get_variableResponse")
(defmethod md5sum ((type (eql '<get_variable-response>)))
  "Returns md5sum for a message object of type '<get_variable-response>"
  "e366705baa8c302f52752de97161f798")
(defmethod message-definition ((type (eql '<get_variable-response>)))
  "Returns full string definition for message of type '<get_variable-response>"
  (format nil "int32 result~%~%"))
(defmethod serialization-length ((msg <get_variable-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <get_variable-response>))
  "Converts a ROS message object to a list"
  (list '<get_variable-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'get_variable)))
  '<get_variable-request>)
(defmethod service-response-type ((msg (eql 'get_variable)))
  '<get_variable-response>)
(defmethod ros-datatype ((msg (eql 'get_variable)))
  "Returns string type for a service object of type '<get_variable>"
  "roscpp_sessions/get_variable")
