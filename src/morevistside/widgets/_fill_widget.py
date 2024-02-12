"""FillWidget inherits space awareness from the SpaceWidget and extends
with a filled background. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QBrush, QPaintEvent, QPainter, QPen
from vistutils import maybe
from vistutils.fields import Field
from vistutils.waitaminute import typeMsg

from morevistside import parseParent
from morevistside.widgets import SpaceWidget


class FillWidget(SpaceWidget):
  """FillWidget inherits space awareness from the SpaceWidget and extends
  with a filled background. """

  __fallback_color__ = Qt.GlobalColor.yellow

  fillColor = Field()
  fillBrush = Field()
  borderPen = Field()

  @backgroundColor.GET
  def getFillColor(self) -> QColor:
    """Getter-function for defined background color"""
    color = maybe(self.__background_color__, self.__fallback_color__)
    if isinstance(color, QColor):
      return color
    e = typeMsg('color', color, QColor)
    raise TypeError(e)

  @fillBrush.GET
  def getFillBrush(self) -> QBrush:
    """Getter-function for filler brush"""
    brush = QBrush()
    brush.setColor(self.fillColor)
    brush.setStyle(Qt.BrushStyle.SolidPattern)
    return brush

  @borderPen.GET
  def getBorderPen(self) -> QPen:
    """Getter-function for the pen drawing the outline around the widget"""
    pen = QPen()
    pen.setStyle(Qt.PenStyle.SolidLine)
    pen.setColor(QColor(0, 0, 0, 255))
    pen.setWidth(2)
    return pen

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    SpaceWidget.__init__(self, parent)
    self.__background_color__ = None
    intArgs = []
    for arg in args:
      if isinstance(arg, QColor):
        self.__background_color__ = arg
        break
      if isinstance(arg, int):
        intArgs.append(arg)
    if self.__background_color__ is None and len(intArgs) > 2:
      self.__background_color__ = QColor(*[*intArgs, 255, ][:4], )

  def paintEvent(self, event: QPaintEvent) -> None:
    """Implementation of paint event covering the viewport area in the
    fill color"""
    painter = QPainter()
    painter.begin(self)
    painter.setBrush(self.fillBrush)
    painter.setPen(self.borderPen)
    viewRect = painter.viewport()
    painter.drawRoundedRect(viewRect, 4, 4)
    painter.end()
