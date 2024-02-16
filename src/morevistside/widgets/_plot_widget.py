"""PlotWidget provides a visual representation of a numerically defined
field."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import logging
import os
import time

from PySide6.QtCore import QPointF, Qt, QMargins, QRectF, Slot
from PySide6.QtGui import QPaintEvent, QPainter, QColor
from PySide6.QtWidgets import QMainWindow
from icecream import ic

from morevistside.factories import solidBrush, parsePen
from morevistside.widgets import FillWidget
from pyros import DataRoll
from rosutils import dataMap

ic.configureOutput(includeContext=True)


class PlotWidget(FillWidget):
  """PlotWidget provides a visual representation of a numerically defined
  field."""

  def __init__(self, parent: QMainWindow, *args, **kwargs) -> None:
    FillWidget.__init__(self, parent)
    self.setMinimumSize(256, 128)
    self.__parent_window__ = parent
    self.__first_paint__ = True
    self.__data_size__ = 128
    self.__x__ = DataRoll(self.__data_size__)
    self.__y__ = DataRoll(self.__data_size__)

  def getParent(self, *args, **kwargs) -> QMainWindow:
    """Getter-function for the parent window"""
    return self.__parent_window__

  @Slot(float, float)
  def receiveValues(self, x: float, y: float) -> None:
    """Receive values for the plot"""
    x = x if x == x else 0
    y = y if y == y else 0
    self.__x__.append(x)
    self.__y__.append(y)

  def getPoints(self, pixelSpace: QRectF) -> list[QPointF]:
    """Getter-function for list of points"""
    pS = pixelSpace
    x0, x1, y0, y1 = [pS.left(), pS.right(), pS.top(), pS.bottom()]
    x_, y_, = self.__x__.getArray(), self.__y__.getArray()
    X, Y = dataMap(x_, x0, x1), dataMap(y_, y0, y1)
    return [QPointF(x, y) for (x, y) in zip(X, Y) if x == x and y == y]

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
