# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from doornumber/board.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class board(genpy.Message):
  _md5sum = "41a76418e981dca789597de9cfa6bfa8"
  _type = "doornumber/board"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int16 tlx
int16 tly
int16 brx
int16 bry
string text
float32 confidence
"""
  __slots__ = ['tlx','tly','brx','bry','text','confidence']
  _slot_types = ['int16','int16','int16','int16','string','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       tlx,tly,brx,bry,text,confidence

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(board, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.tlx is None:
        self.tlx = 0
      if self.tly is None:
        self.tly = 0
      if self.brx is None:
        self.brx = 0
      if self.bry is None:
        self.bry = 0
      if self.text is None:
        self.text = ''
      if self.confidence is None:
        self.confidence = 0.
    else:
      self.tlx = 0
      self.tly = 0
      self.brx = 0
      self.bry = 0
      self.text = ''
      self.confidence = 0.

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
      buff.write(_struct_4h.pack(_x.tlx, _x.tly, _x.brx, _x.bry))
      _x = self.text
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_f.pack(self.confidence))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.tlx, _x.tly, _x.brx, _x.bry,) = _struct_4h.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.text = str[start:end].decode('utf-8')
      else:
        self.text = str[start:end]
      start = end
      end += 4
      (self.confidence,) = _struct_f.unpack(str[start:end])
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
      _x = self
      buff.write(_struct_4h.pack(_x.tlx, _x.tly, _x.brx, _x.bry))
      _x = self.text
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_f.pack(self.confidence))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.tlx, _x.tly, _x.brx, _x.bry,) = _struct_4h.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.text = str[start:end].decode('utf-8')
      else:
        self.text = str[start:end]
      start = end
      end += 4
      (self.confidence,) = _struct_f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4h = struct.Struct("<4h")
_struct_f = struct.Struct("<f")
