"""DataType provides a dataclass representation of a type with a default
value. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
from typing import Any


class DataType:
  """Data class representation of type with default value"""

  __slots__ = ['type', 'defVal']
  __common_defaults__ = {int : 0, float: .0, complex: 0j, str: '',
                         bool: False, list: [],
                         dict: dict(), set: set(), }

  @classmethod
  def _getCommonDefault(cls, ) -> dict:
    """Returns a dictionary where keys are types and values are suggested
    default values"""
    return cls.__common_defaults__

  @classmethod
  def _parse(cls, *args, **_) -> Any:
    """Parses positional arguments"""
    if len(args) == 2:
      if not isinstance(args[0], type):
        if isinstance(args[1], type):
          return cls._parse(args[1], args[0])
        raise TypeError
      if not isinstance(args[1], args[0]):
        raise TypeError
      return (*args,)
    if len(args) == 1:
      if isinstance(args[0], type):
        commonDefaults = cls._getCommonDefault()
        defVal = commonDefaults.get(args[0], None)
        if defVal is not None:
          return cls._parse(args[0], defVal)
        defVal = args[0]()
        return cls._parse(args[0], defVal)
      return (type(args[0]), args[0])

  def __init__(self, *args) -> None:
    parsed = self._parse(*args)
    if parsed is None:
      raise ValueError
    parsed = (*parsed,)[:2]
    self.type, self.defVal = parsed
