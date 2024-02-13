"""PlotWidget provides a visual representation of a numerically defined
field."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import TYPE_CHECKING

from PySide6.QtCore import QPointF, Qt, QRect, QMargins, QRectF
from PySide6.QtGui import QPaintEvent, QPainter, QPen, QColor, QBrush
from PySide6.QtWidgets import QWidget
from icecream import ic

if TYPE_CHECKING:
  from morevistside.windows import MainWindow

ic.configureOutput(includeContext=True)


class PlotWidget(QWidget):
  """PlotWidget provides a visual representation of a numerically defined
  field."""

  def __init__(self, parent: MainWindow, *args, **kwargs) -> None:
    QWidget.__init__(self, parent)
    self.setMinimumSize(256, 128)
    self.__parent_window__ = parent
    self.__first_paint__ = True

  def getParent(self, *args, **kwargs) -> MainWindow:
    """Getter-function for the parent window"""
    return self.__parent_window__

  def getPoints(self, pixelSpace: QRectF) -> list[QPointF]:
    """Getter-function for list of points"""
    parent = self.getParent()
    return parent.data.getPoints(pixelSpace)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Implementation of paint event"""
    # if not self._unPaintedChanges:
    #   return QWidget.paintEvent(self, event)
    # self._unPaintedChanges = False
    tic = time.time()
    painter = QPainter()
    painter.begin(self)
    backgroundBrush = QBrush()
    backgroundBrush.setStyle(Qt.BrushStyle.SolidPattern)
    backgroundBrush.setColor(QColor(127, 255, 0, 255))
    pointPen = QPen()
    pointPen.setWidth(5)
    pointPen.setColor(QColor(0, 0, 0, 255))
    pointPen.setStyle(Qt.PenStyle.SolidLine)
    painter.setPen(pointPen)
    painter.setBrush(backgroundBrush)
    viewRect = painter.viewport()
    marginRect = viewRect - QMargins(10, 10, 10, 10, )
    painter.drawRect(viewRect, )
    painter.drawText(marginRect, time.ctime())
    points = self.getPoints(marginRect)
    painter.drawPoints(points)
    painter.end()
