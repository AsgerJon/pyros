"""The ColorField class provides descriptor access to color specifications"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtGui import QColor
from vistutils.fields import AbstractField
from vistutils.waitaminute import typeMsg

from morevistside.widgets import Color


class ColorField(AbstractField):
  """The ColorField class provides descriptor access to color
  specifications"""

  def __prepare_owner__(self, owner: type) -> type:
    """Implementation of abstract method"""
    return owner

  def __init__(self, *args, **kwargs) -> None:
    AbstractField.__init__(self, *args, **kwargs)
    self.__default_value__ = Color(*args)

  def __get__(self, instance: Any, owner: type, **kwargs) -> Color:
    """Getter-function"""
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      color = getattr(instance, pvtName)
      if isinstance(color, Color):
        return color
      e = typeMsg('color', color, Color)
      raise TypeError
    if kwargs.get('_recursion', False):
      raise RecursionError
    setattr(instance, pvtName, self.__default_value__)
    return self.__get__(instance, owner, _recursion=True, **kwargs)

  def __set__(self, instance: Any, color: Any) -> None:
    """Setter-function"""
    pvtName = self._getPrivateName()
    if isinstance(color, Color):
      return setattr(instance, pvtName, color)
    if isinstance(color, QColor):
      return setattr(instance, pvtName, Color.fromQColor(color))
