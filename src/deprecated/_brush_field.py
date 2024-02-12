"""BrushField provides a special descriptor class pointing to instances of
QBrush"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush
from vistutils.fields import AbstractField

from deprecated.widgets import ColorField


class BrushField(AbstractField):
  """BrushField provides a special descriptor class pointing to instances of
  QBrush"""

  color = ColorField(127, 255, 0, 255)

  def __prepare_owner__(self, owner: type) -> type:
    """Implementation of abstract method"""
    return owner

  def __get__(self, instance: Any, owner: type) -> QBrush:
    """Returns a QBrush with inner color"""
    brush = QBrush()
    brush.setStyle(Qt.BrushStyle.SolidPattern)
    brush.setColor(self.color.asQColor)
    return brush

  def __set__(self, instance: Any, value: Any) -> None:
    """Sets appropriate value on instance"""
