"""Tester class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic

ic.configureOutput(includeContext=True)


class SomeClass:
  """Yolo"""

  def __new__(cls, *args, **kwargs) -> SomeClass:
    if '_root' in kwargs:
      newKwargs = {k: v for (k, v) in kwargs.items() if k != '_root'}
      self = SomeClass.__new__(cls, *args, **newKwargs)
      return self
    else:
      self = object.__new__(cls)
    self.__new_args__ = args
    self.__new_kwargs__ = kwargs
    return self

  def __init__(self, *args, **kwargs) -> None:
    self.__init_args__ = args
    self.__init_kwargs__ = kwargs
    ic(self.__new_args__ == self.__init_args__)
    ic(self.__new_kwargs__ == self.__init_kwargs__)
