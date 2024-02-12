"""FillWidget is a subclass of BaseWidget implementing a filled background
in the paint event."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPaintEvent, QBrush, QPainter
from vistutils import monoSpace
from vistutils.fields import Field

from deprecated.widgets import BaseWidget


class FillWidget(BaseWidget):
  """FillWidget is a subclass of BaseWidget implementing a filled background
  in the paint event."""

  fillColor = Field()
  fillBrush = Field()

  @fillColor.GET
  def _getFillColor(self) -> QColor:
    """Getter-function for fill color"""
    return self.__fill_color__

  @fillColor.SET
  def _setFillColor(self, color: QColor) -> None:
    """Getter-function for fill color"""
    self.__fill_color__ = color

  @fillBrush.GET
  def _getFillBrush(self) -> QBrush:
    """Getter-function for the brush filling the background"""
    brush = QBrush()
    brush.setStyle(Qt.BrushStyle.SolidPattern)
    brush.setColor(self.fillColor)
    return brush

  @staticmethod
  def _parseQColor(*args) -> Optional[QColor]:
    """Parses positional arguments returning the first QColor"""
    for arg in args:
      if isinstance(arg, QColor):
        return arg

  def __init__(self, *args, **kwargs) -> None:
    BaseWidget.__init__(self, *args, **kwargs)
    color = self._parseQColor(*args)
    if color is None:
      e = """Instances of FillWidget and subclasses must be provided a 
      QColor as a positional argument."""
      raise ValueError(monoSpace(e))
    self.__fill_color__ = color

  def paintEvent(self, evnet: QPaintEvent) -> None:
    """Fills the background with given color"""
    painter = QPainter()
    painter.begin(self, )
    viewRect = painter.viewport()
    painter.setBrush(self.fillBrush)
    painter.drawRoundedRect(viewRect, 4, 4, )
    painter.end()
