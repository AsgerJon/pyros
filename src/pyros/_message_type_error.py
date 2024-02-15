"""MessageTypeError is a custom exception intended to describe the
situation where a message is published to a RosTopic with an unsupported
type."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils.waitaminute import typeMsg


class MessageTypeError(TypeError):
  """MessageTypeError is a custom exception intended to describe the
  situation where a message is published to a RosTopic with an unsupported
  type."""

  def __init__(self, *args) -> None:
    name = None
    obj = None
    expType = None
    if len(args) == 1:
      if isinstance(args[0], str):
        TypeError.__init__(self, args[0])
    elif len(args) > 2:
      for arg in args:
        if isinstance(arg, str) and name is None:
          name = arg
        elif isinstance(arg, type) and expType is None:
          expType = arg
        elif obj is None:
          obj = arg
      if all([arg is not None for arg in [name, obj, expType]]):
        e = typeMsg(name, obj, expType)
        TypeError.__init__(self, e)
      else:
        TypeError.__init__(self, )
    else:
      TypeError.__init__(self, )
