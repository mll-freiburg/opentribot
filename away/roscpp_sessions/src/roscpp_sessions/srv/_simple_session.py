"""autogenerated by genmsg_py from simple_sessionRequest.msg. Do not edit."""
import roslib.message
import struct


class simple_sessionRequest(roslib.message.Message):
  _md5sum = "356582ab258a918ca4a809d930432d02"
  _type = "roscpp_sessions/simple_sessionRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 sessionid
int32 options

"""
  __slots__ = ['sessionid','options']
  _slot_types = ['int32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       sessionid,options
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(simple_sessionRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.sessionid is None:
        self.sessionid = 0
      if self.options is None:
        self.options = 0
    else:
      self.sessionid = 0
      self.options = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_2i.pack(_x.sessionid, _x.options))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.sessionid, _x.options,) = _struct_2i.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_2i.pack(_x.sessionid, _x.options))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.sessionid, _x.options,) = _struct_2i.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2i = struct.Struct("<2i")
"""autogenerated by genmsg_py from simple_sessionResponse.msg. Do not edit."""
import roslib.message
import struct


class simple_sessionResponse(roslib.message.Message):
  _md5sum = "8e5dc427c491661ad151bd29bea23253"
  _type = "roscpp_sessions/simple_sessionResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 sessionid


"""
  __slots__ = ['sessionid']
  _slot_types = ['int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       sessionid
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(simple_sessionResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.sessionid is None:
        self.sessionid = 0
    else:
      self.sessionid = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      buff.write(_struct_i.pack(self.sessionid))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 4
      (self.sessionid,) = _struct_i.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      buff.write(_struct_i.pack(self.sessionid))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      start = end
      end += 4
      (self.sessionid,) = _struct_i.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_i = struct.Struct("<i")
class simple_session(roslib.message.ServiceDefinition):
  _type          = 'roscpp_sessions/simple_session'
  _md5sum = '42123af46840487b5635bddfd445b0b1'
  _request_class  = simple_sessionRequest
  _response_class = simple_sessionResponse
