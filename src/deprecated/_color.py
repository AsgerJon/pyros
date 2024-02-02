"""Color provides a descriptor friendly dataclass like implementation of
colors."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QColor

from morevistutils.fields import TypedField


class Color:
  """Color dataclass"""

  red = TypedField(int, 255)
  green = TypedField(int, 255)
  blue = TypedField(int, 255)
  alpha = TypedField(int, 255)

  @classmethod
  def fromQColor(cls, color: QColor) -> Color:
    """Creates an instance of color from the QColor"""
    r, g, b, a = [color.red(), color.green(), color.blue(), color.alpha(), ]
    return cls(r, g, b, a)

  def __init__(self, *args, **kwargs) -> None:
    self.__iter_contents__ = []

  def asQColor(self) -> QColor:
    """Casts the color as instance of QColor"""
    return QColor(*self)

  def __iter__(self, ) -> Color:
    """Implementation of iteration"""
    self.__iter_contents__ = [self.red, self.green, self.blue, self.alpha]
    return self

  def __next__(self, ) -> int:
    """Implementation of iteration"""
    try:
      return self.__iter_contents__.pop(0)
    except IndexError as indexError:
      raise StopIteration
