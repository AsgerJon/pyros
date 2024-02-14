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
from morevistside.factories import solidBrush, parsePen
from morevistside.widgets import SpaceWidget


class FillWidget(SpaceWidget):
  """FillWidget inherits space awareness from the SpaceWidget and extends
  with a filled background. """

  __fallback_color__ = QColor(255, 255, 0, 255)

  fillColor = Field()
  fillBrush = Field()
  borderPen = Field()

  @fillColor.GET
  def getFillColor(self) -> QColor:
    """Getter-function for defined background color"""
    color = maybe(self.__background_color__, self.__fallback_color__)
    if isinstance(color, QColor):
      return color
    e = typeMsg('color', color, QColor)
    raise TypeError(e)

  @fillBrush.GET
  def getFillBrush(self, **kwargs) -> QBrush:
    """Getter-function for filler brush"""
    if hasattr(self, '_fillBrush'):
      return self._fillBrush
    if kwargs.get('_recursion', False):
      raise RecursionError
    setattr(self, '_fillBrush', solidBrush(self.fillColor))
    return self.getFillBrush(_recursion=True)

  @borderPen.GET
  def getBorderPen(self, **kwargs) -> QPen:
    """Getter-function for the pen drawing the outline around the widget"""
    if hasattr(self, '_borderPen'):
      if isinstance(self._borderPen, QPen):
        return self._borderPen
      e = typeMsg('_borderPen', self._borderPen, QPen)
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    borderColor = QColor(0, 0, 0, 255)
    borderWidth = 2
    borderStyle = Qt.PenStyle.SolidLine
    borderPen = parsePen(borderColor, borderWidth, borderStyle)
    setattr(self, '_borderPen', borderPen)
    return self.getBorderPen(_recursion=True)

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    sizes = dict(minHeight=64, maxHeight=512, minWidth=64, maxWidth=512, )
    SpaceWidget.__init__(self, parent, **(sizes | kwargs))
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
