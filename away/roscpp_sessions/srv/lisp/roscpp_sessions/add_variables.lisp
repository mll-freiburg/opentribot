; Auto-generated. Do not edit!


(in-package roscpp_sessions-srv)


;//! \htmlinclude add_variables-request.msg.html

(defclass <add_variables-request> (ros-message)
  ((variable1
    :reader variable1-val
    :initarg :variable1
    :type string
    :initform "")
   (variable2
    :reader variable2-val
    :initarg :variable2
    :type string
    :initform "")
   (result
    :reader result-val
    :initarg :result
    :type string
    :initform ""))
)
(defmethod serialize ((msg <add_variables-request>) ostream)
  "Serializes a message object of type '<add_variables-request>"
  (let ((__ros_str_len (length (slot-value msg 'variable1))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'variable1))
  (let ((__ros_str_len (length (slot-value msg 'variable2))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'variable2))
  (let ((__ros_str_len (length (slot-value msg 'result))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'result))
)
(defmethod deserialize ((msg <add_variables-request>) istream)
  "Deserializes a message object of type '<add_variables-request>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'variable1) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'variable1) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'variable2) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'variable2) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'result) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'result) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<add_variables-request>)))
  "Returns string type for a service object of type '<add_variables-request>"
  "roscpp_sessions/add_variablesRequest")
(defmethod md5sum ((type (eql '<add_variables-request>)))
  "Returns md5sum for a message object of type '<add_variables-request>"
  "d14cc3cb09d105ec5c963edd2748d6fb")
(defmethod message-definition ((type (eql '<add_variables-request>)))
  "Returns full string definition for message of type '<add_variables-request>"
  (format nil "string variable1~%string variable2~%string result~%~%"))
(defmethod serialization-length ((msg <add_variables-request>))
  (+ 0
     4 (length (slot-value msg 'variable1))
     4 (length (slot-value msg 'variable2))
     4 (length (slot-value msg 'result))
))
(defmethod ros-message-to-list ((msg <add_variables-request>))
  "Converts a ROS message object to a list"
  (list '<add_variables-request>
    (cons ':variable1 (variable1-val msg))
    (cons ':variable2 (variable2-val msg))
    (cons ':result (result-val msg))
))
;//! \htmlinclude add_variables-response.msg.html

(defclass <add_variables-response> (ros-message)
  ()
)
(defmethod serialize ((msg <add_variables-response>) ostream)
  "Serializes a message object of type '<add_variables-response>"
)
(defmethod deserialize ((msg <add_variables-response>) istream)
  "Deserializes a message object of type '<add_variables-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<add_variables-response>)))
  "Returns string type for a service object of type '<add_variables-response>"
  "roscpp_sessions/add_variablesResponse")
(defmethod md5sum ((type (eql '<add_variables-response>)))
  "Returns md5sum for a message object of type '<add_variables-response>"
  "d14cc3cb09d105ec5c963edd2748d6fb")
(defmethod message-definition ((type (eql '<add_variables-response>)))
  "Returns full string definition for message of type '<add_variables-response>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <add_variables-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <add_variables-response>))
  "Converts a ROS message object to a list"
  (list '<add_variables-response>
))
(defmethod service-request-type ((msg (eql 'add_variables)))
  '<add_variables-request>)
(defmethod service-response-type ((msg (eql 'add_variables)))
  '<add_variables-response>)
(defmethod ros-datatype ((msg (eql 'add_variables)))
  "Returns string type for a service object of type '<add_variables>"
  "roscpp_sessions/add_variables")
