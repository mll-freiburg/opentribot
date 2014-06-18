; Auto-generated. Do not edit!


(in-package roscpp_sessions-srv)


;//! \htmlinclude set_variable-request.msg.html

(defclass <set_variable-request> (ros-message)
  ((variable
    :reader variable-val
    :initarg :variable
    :type string
    :initform "")
   (value
    :reader value-val
    :initarg :value
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <set_variable-request>) ostream)
  "Serializes a message object of type '<set_variable-request>"
  (let ((__ros_str_len (length (slot-value msg 'variable))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'variable))
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'value)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'value)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <set_variable-request>) istream)
  "Deserializes a message object of type '<set_variable-request>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'variable) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'variable) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'value)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'value)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<set_variable-request>)))
  "Returns string type for a service object of type '<set_variable-request>"
  "roscpp_sessions/set_variableRequest")
(defmethod md5sum ((type (eql '<set_variable-request>)))
  "Returns md5sum for a message object of type '<set_variable-request>"
  "93d6cd2ef0527c532b42de2a1705eb38")
(defmethod message-definition ((type (eql '<set_variable-request>)))
  "Returns full string definition for message of type '<set_variable-request>"
  (format nil "string variable~%int32 value~%~%"))
(defmethod serialization-length ((msg <set_variable-request>))
  (+ 0
     4 (length (slot-value msg 'variable))
     4
))
(defmethod ros-message-to-list ((msg <set_variable-request>))
  "Converts a ROS message object to a list"
  (list '<set_variable-request>
    (cons ':variable (variable-val msg))
    (cons ':value (value-val msg))
))
;//! \htmlinclude set_variable-response.msg.html

(defclass <set_variable-response> (ros-message)
  ()
)
(defmethod serialize ((msg <set_variable-response>) ostream)
  "Serializes a message object of type '<set_variable-response>"
)
(defmethod deserialize ((msg <set_variable-response>) istream)
  "Deserializes a message object of type '<set_variable-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<set_variable-response>)))
  "Returns string type for a service object of type '<set_variable-response>"
  "roscpp_sessions/set_variableResponse")
(defmethod md5sum ((type (eql '<set_variable-response>)))
  "Returns md5sum for a message object of type '<set_variable-response>"
  "93d6cd2ef0527c532b42de2a1705eb38")
(defmethod message-definition ((type (eql '<set_variable-response>)))
  "Returns full string definition for message of type '<set_variable-response>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <set_variable-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <set_variable-response>))
  "Converts a ROS message object to a list"
  (list '<set_variable-response>
))
(defmethod service-request-type ((msg (eql 'set_variable)))
  '<set_variable-request>)
(defmethod service-response-type ((msg (eql 'set_variable)))
  '<set_variable-response>)
(defmethod ros-datatype ((msg (eql 'set_variable)))
  "Returns string type for a service object of type '<set_variable>"
  "roscpp_sessions/set_variable")
