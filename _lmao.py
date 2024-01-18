"""LMAO StringIO subclass"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from io import StringIO
from typing import Optional

from utils.text import stringList


class LMAO(StringIO):
  """LMAO StringIO subclass"""

  @staticmethod
  def _parseArgs(*args, ) -> Optional[str]:
    """Parses positional arguments"""
    for arg in args:
      if isinstance(arg, str):
        return arg

  @staticmethod
  def _parseKwargs(**kwargs) -> Optional[str]:
    """Parses keyword arguments"""
    initKeys = stringList("""first, start, begin, init, initialValue, 
    initial_value""")
    for key in [k for k in initKeys if k in kwargs]:
      val = kwargs.get(key, )
      if isinstance(val, str):
        return val

  @classmethod
  def _parse(cls, *args, **kwargs) -> Optional[str]:
    """Parses arguments"""
    initKwarg = cls._parseKwargs(**kwargs)
    initArg = cls._parseArgs(*args)
    initDefault = ''
    return maybe(initKwarg, initArg, initDefault)

  def __init__(self, *args, **kwargs) -> None:
    strArgs = [arg for arg in args if isinstance(arg, str)]
    initialValue = [*strArgs, None]

    StringIO.__init__(self, initial_value='', *args, **kwargs)
