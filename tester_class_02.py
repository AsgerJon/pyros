"""TESTER"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from vistutils.fields import Field


class ShareField:
  """LOL"""

  @staticmethod
  def yolo(callMeMaybe: Callable) -> Callable:
    """Sets the getter"""
    # print('yolo @ %s' % callMeMaybe.__name__)
    return callMeMaybe

  a = Field()  # Custom descriptor class
  b = Field()  # Custom descriptor class

  @a.GET
  @b.GET
  def _getAorB(self, **kwargs) -> Any:
    """Getter-function for the a"""
    if hasattr(self, 'ab'):
      return getattr(self, 'ab')
    if kwargs.get('_recursion', False):
      raise RecursionError
    setattr(self, 'ab', 'LMAO')
    return self._getAorB(_recursion=True)

  @yolo
  def __str__(self) -> str:
    """String representation of ShareField"""
    return f"""ShareField(a={self.a}, b={self.b})"""


class ShareChild(ShareField):
  """LOL"""

  @ShareField.yolo
  def __getattribute__(self, key: str) -> Any:
    """Getter for the attribute"""
    return object.__getattribute__(self, key)
