# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from opencv_apps/RectArray.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import opencv_apps.msg

class RectArray(genpy.Message):
  _md5sum = "d4a6f20c7699fa2791af675958a5f148"
  _type = "opencv_apps/RectArray"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Rect[] rects

================================================================================
MSG: opencv_apps/Rect
# opencv Rect data type, x-y is center point
float64 x
float64 y
float64 width
float64 height

"""
  __slots__ = ['rects']
  _slot_types = ['opencv_apps/Rect[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       rects

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(RectArray, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.rects is None:
        self.rects = []
    else:
      self.rects = []

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
      length = len(self.rects)
      buff.write(_struct_I.pack(length))
      for val1 in self.rects:
        _x = val1
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.width, _x.height))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.rects is None:
        self.rects = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.rects = []
      for i in range(0, length):
        val1 = opencv_apps.msg.Rect()
        _x = val1
        start = end
        end += 32
        (_x.x, _x.y, _x.width, _x.height,) = _struct_4d.unpack(str[start:end])
        self.rects.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.rects)
      buff.write(_struct_I.pack(length))
      for val1 in self.rects:
        _x = val1
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.width, _x.height))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.rects is None:
        self.rects = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.rects = []
      for i in range(0, length):
        val1 = opencv_apps.msg.Rect()
        _x = val1
        start = end
        end += 32
        (_x.x, _x.y, _x.width, _x.height,) = _struct_4d.unpack(str[start:end])
        self.rects.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4d = struct.Struct("<4d")
