"""ColorDescriptor provides a QColor valued subclass of AbstractStencil"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Any, Never

from PySide6.QtGui import QColor
from vistutils import maybe

from morevistside import parseHex
from morevistside.paintmelike import AbstractStencil


class ColorDescriptor(AbstractStencil):
  """ColorDescriptor defines the colored background on the widget"""

  __default_color__ = os.environ.get('DEFAULT_BACKGROUND_COLOR')

  def __get__(self, instance: Any, owner: type) -> QColor:
    """Implementation of getter function"""
    pvtName = self._getPrivateName()
    if instance is None:
      return getattr(owner, '__default_background__')
    if hasattr(instance, pvtName):
      color = getattr(instance, pvtName)
      if isinstance(color, QColor):
        return color
      if isinstance(color, str):
        return parseHex(color)
      raise TypeError('color', color, QColor)
    ownerDefault = getattr(owner, '__default_color__', None)
    descriptorDefault = getattr(self, '__default_color__', None)
    defVal = maybe(ownerDefault, descriptorDefault)
    setattr(instance, pvtName, defVal)
    if isinstance(defVal, QColor):
      return defVal
    return parseHex(defVal)

  def __set__(self, instance: Any, value: Any) -> None:
    """Setter-function for color"""
    color = parseHex(value)
    pvtName = self._getPrivateName()
    setattr(instance, pvtName, color)

  def __delete__(self, *_) -> Never:
    """Illegal deleter function"""
    e = """Color descriptor does not implement deletion!"""
    raise TypeError(e)
