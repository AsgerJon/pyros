"""PlotWidget provides a visual representation of a numerically defined
field."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

from PySide6.QtCore import QPointF, Qt, QMargins, QRectF
from PySide6.QtGui import QPaintEvent, QPainter, QPen, QColor, QBrush
from PySide6.QtWidgets import QMainWindow
from icecream import ic

from morevistside.factories import solidBrush, parsePen
from morevistside.widgets import FillWidget
from morevistutils import DataArray
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class PlotWidget(FillWidget):
  """PlotWidget provides a visual representation of a numerically defined
  field."""

  def __init__(self, parent: QMainWindow, *args, **kwargs) -> None:
    FillWidget.__init__(self, parent)
    self.setMinimumSize(256, 128)
    self.__parent_window__ = parent
    self.__first_paint__ = True

  def getParent(self, *args, **kwargs) -> QMainWindow:
    """Getter-function for the parent window"""
    return self.__parent_window__

  def getPoints(self, pixelSpace: QRectF) -> list[QPointF]:
    """Getter-function for list of points"""
    parent = self.getParent()
    if hasattr(parent, 'data'):
      data = getattr(parent, 'data')
      if isinstance(data, DataArray):
        return data.getPoints(pixelSpace)
      e = typeMsg('data', data, DataArray)
      raise TypeError(e)
    e = """Expected parent window to provide an attribute at name: 'data'!"""
    raise AttributeError(e)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Implementation of paint event"""
    # if not self._unPaintedChanges:
    #   return QWidget.paintEvent(self, event)
    # self._unPaintedChanges = False
    painter = QPainter()
    painter.begin(self)
    backgroundBrush = solidBrush(QColor(127, 255, 0, 255))
    pointPen = parsePen(QColor(0, 0, 0, 255), 5, Qt.PenStyle.SolidLine)
    painter.setPen(pointPen)
    painter.setBrush(backgroundBrush)
    viewRect = painter.viewport()
    marginRect = viewRect - QMargins(10, 10, 10, 10, )
    painter.drawRect(viewRect, )
    painter.drawText(marginRect, time.ctime())
    points = self.getPoints(marginRect)
    painter.drawPoints(points)
    painter.end()
