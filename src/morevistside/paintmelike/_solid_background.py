"""BackgroundStencil defines the colored background on the widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QPaintEvent, QBrush

from morevistside.paintmelike import ColorDescriptor
from morevistutils.waitaminute import typeMsg


class SolidBackground(ColorDescriptor):
  """Paints a solid background of the color defined on the parent class."""

  def _getBrush(self, instance: Any) -> QBrush:
    """Getter-function for QBrush"""
    brush = QBrush()
    brush.setStyle(Qt.BrushStyle.SolidPattern)
    brush.setColor(self.__get__(instance, self._getFieldOwner()))
    return brush

  def paintMeLike(self, painter: QPainter, event: QPaintEvent) -> None:
    """The painter should be activated and pointing to a device belong to
    the owning class. """
    ColorDescriptor.paintMeLike(self, painter, event)
    device = painter.device()
    painter.setBrush(self._getBrush(device))
    painter.setPen(self._getEmptyPen())
    painter.drawRect(event.rect())
