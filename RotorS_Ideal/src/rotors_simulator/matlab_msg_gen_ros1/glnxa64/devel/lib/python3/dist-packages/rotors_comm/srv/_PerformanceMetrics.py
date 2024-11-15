# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rotors_comm/PerformanceMetricsRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class PerformanceMetricsRequest(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "rotors_comm/PerformanceMetricsRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PerformanceMetricsRequest, self).__init__(*args, **kwds)

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rotors_comm/PerformanceMetricsResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class PerformanceMetricsResponse(genpy.Message):
  _md5sum = "489ff84073d2b57991c40f5769f49311"
  _type = "rotors_comm/PerformanceMetricsResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64 overshoot1
float64 overshoot2
float64 overshoot3
float64 overshoot4
float64 overshoot5
float64 overshoot6
float64 settling_time1
float64 settling_time2
float64 settling_time3
float64 settling_time4
float64 settling_time5
float64 settling_time6

"""
  __slots__ = ['overshoot1','overshoot2','overshoot3','overshoot4','overshoot5','overshoot6','settling_time1','settling_time2','settling_time3','settling_time4','settling_time5','settling_time6']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       overshoot1,overshoot2,overshoot3,overshoot4,overshoot5,overshoot6,settling_time1,settling_time2,settling_time3,settling_time4,settling_time5,settling_time6

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PerformanceMetricsResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.overshoot1 is None:
        self.overshoot1 = 0.
      if self.overshoot2 is None:
        self.overshoot2 = 0.
      if self.overshoot3 is None:
        self.overshoot3 = 0.
      if self.overshoot4 is None:
        self.overshoot4 = 0.
      if self.overshoot5 is None:
        self.overshoot5 = 0.
      if self.overshoot6 is None:
        self.overshoot6 = 0.
      if self.settling_time1 is None:
        self.settling_time1 = 0.
      if self.settling_time2 is None:
        self.settling_time2 = 0.
      if self.settling_time3 is None:
        self.settling_time3 = 0.
      if self.settling_time4 is None:
        self.settling_time4 = 0.
      if self.settling_time5 is None:
        self.settling_time5 = 0.
      if self.settling_time6 is None:
        self.settling_time6 = 0.
    else:
      self.overshoot1 = 0.
      self.overshoot2 = 0.
      self.overshoot3 = 0.
      self.overshoot4 = 0.
      self.overshoot5 = 0.
      self.overshoot6 = 0.
      self.settling_time1 = 0.
      self.settling_time2 = 0.
      self.settling_time3 = 0.
      self.settling_time4 = 0.
      self.settling_time5 = 0.
      self.settling_time6 = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_12d().pack(_x.overshoot1, _x.overshoot2, _x.overshoot3, _x.overshoot4, _x.overshoot5, _x.overshoot6, _x.settling_time1, _x.settling_time2, _x.settling_time3, _x.settling_time4, _x.settling_time5, _x.settling_time6))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 96
      (_x.overshoot1, _x.overshoot2, _x.overshoot3, _x.overshoot4, _x.overshoot5, _x.overshoot6, _x.settling_time1, _x.settling_time2, _x.settling_time3, _x.settling_time4, _x.settling_time5, _x.settling_time6,) = _get_struct_12d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_12d().pack(_x.overshoot1, _x.overshoot2, _x.overshoot3, _x.overshoot4, _x.overshoot5, _x.overshoot6, _x.settling_time1, _x.settling_time2, _x.settling_time3, _x.settling_time4, _x.settling_time5, _x.settling_time6))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 96
      (_x.overshoot1, _x.overshoot2, _x.overshoot3, _x.overshoot4, _x.overshoot5, _x.overshoot6, _x.settling_time1, _x.settling_time2, _x.settling_time3, _x.settling_time4, _x.settling_time5, _x.settling_time6,) = _get_struct_12d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_12d = None
def _get_struct_12d():
    global _struct_12d
    if _struct_12d is None:
        _struct_12d = struct.Struct("<12d")
    return _struct_12d
class PerformanceMetrics(object):
  _type          = 'rotors_comm/PerformanceMetrics'
  _md5sum = '489ff84073d2b57991c40f5769f49311'
  _request_class  = PerformanceMetricsRequest
  _response_class = PerformanceMetricsResponse
