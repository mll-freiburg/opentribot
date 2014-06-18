; Auto-generated. Do not edit!


(in-package roscpp_sessions-srv)


;//! \htmlinclude simple_session-request.msg.html

(defclass <simple_session-request> (ros-message)
  ((sessionid
    :reader sessionid-val
    :initarg :sessionid
    :type integer
    :initform 0)
   (options
    :reader options-val
    :initarg :options
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <simple_session-request>) ostream)
  "Serializes a message object of type '<simple_session-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'sessionid)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'sessionid)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'sessionid)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'sessionid)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'options)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'options)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'options)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'options)) ostream)
)
(defmethod deserialize ((msg <simple_session-request>) istream)
  "Deserializes a message object of type '<simple_session-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'sessionid)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'sessionid)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'sessionid)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'sessionid)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'options)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'options)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'options)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'options)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<simple_session-request>)))
  "Returns string type for a service object of type '<simple_session-request>"
  "roscpp_sessions/simple_sessionRequest")
(defmethod md5sum ((type (eql '<simple_session-request>)))
  "Returns md5sum for a message object of type '<simple_session-request>"
  "42123af46840487b5635bddfd445b0b1")
(defmethod message-definition ((type (eql '<simple_session-request>)))
  "Returns full string definition for message of type '<simple_session-request>"
  (format nil "int32 sessionid~%int32 options~%~%"))
(defmethod serialization-length ((msg <simple_session-request>))
  (+ 0
     4
     4
))
(defmethod ros-message-to-list ((msg <simple_session-request>))
  "Converts a ROS message object to a list"
  (list '<simple_session-request>
    (cons ':sessionid (sessionid-val msg))
    (cons ':options (options-val msg))
))
;//! \htmlinclude simple_session-response.msg.html

(defclass <simple_session-response> (ros-message)
  ((sessionid
    :reader sessionid-val
    :initarg :sessionid
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <simple_session-response>) ostream)
  "Serializes a message object of type '<simple_session-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'sessionid)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'sessionid)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'sessionid)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'sessionid)) ostream)
)
(defmethod deserialize ((msg <simple_session-response>) istream)
  "Deserializes a message object of type '<simple_session-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'sessionid)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'sessionid)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'sessionid)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'sessionid)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<simple_session-response>)))
  "Returns string type for a service object of type '<simple_session-response>"
  "roscpp_sessions/simple_sessionResponse")
(defmethod md5sum ((type (eql '<simple_session-response>)))
  "Returns md5sum for a message object of type '<simple_session-response>"
  "42123af46840487b5635bddfd445b0b1")
(defmethod message-definition ((type (eql '<simple_session-response>)))
  "Returns full string definition for message of type '<simple_session-response>"
  (format nil "int32 sessionid~%~%~%"))
(defmethod serialization-length ((msg <simple_session-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <simple_session-response>))
  "Converts a ROS message object to a list"
  (list '<simple_session-response>
    (cons ':sessionid (sessionid-val msg))
))
(defmethod service-request-type ((msg (eql 'simple_session)))
  '<simple_session-request>)
(defmethod service-response-type ((msg (eql 'simple_session)))
  '<simple_session-response>)
(defmethod ros-datatype ((msg (eql 'simple_session)))
  "Returns string type for a service object of type '<simple_session>"
  "roscpp_sessions/simple_session")
