"""AxesLayer draws axes on the widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt, QPointF, QLineF
from PySide6.QtGui import QPen, QColor, QPainter

from morevistside.paintlayers import AbstractPaintLayer


class AxesLayer(AbstractPaintLayer):
  """AxesLayer draws axes on the widget"""

  def __init__(self, *args, **kwargs) -> None:
    AbstractPaintLayer.__init__(self, *args, **kwargs)
    x0, y0 = None, None
    x0, y0 = kwargs.get('x0', None), kwargs.get('y0', None)
    for arg in args:
      if isinstance(arg, float):
        if x0 is None:
          x0 = arg
        elif y0 is None:
          y0 = arg
      if isinstance(arg, complex) and x0 is None and y0 is None:
        x0, y0 = arg.real, arg.imag
    if x0 is None:
      x0 = .5
    if y0 is None:
      y0 = .5

    self._x0, self._y0 = x0, y0
    _pen = QPen()
    _pen.setWidthF(1)
    _pen.setColor(QColor(0, 0, 0, 255))
    _pen.setStyle(Qt.PenStyle.DashLine)
    self._pen = _pen

  def paintMeLike(self, painter: QPainter) -> None:
    """This implementation makes use of the QPainterPath"""
    device = painter.device()
    viewRect = painter.viewport()
    width, height = viewRect.width(), viewRect.height()
    vLineTop = QPointF(self._x0 * width, 0)
    vLineBottom = QPointF(self._x0 * width, height)
    vLine = QLineF(vLineTop, vLineBottom)
    hLineTop = QPointF(0, self._y0 * height, )
    hLineBottom = QPointF(width, self._y0 * height, )
    hLine = QLineF(hLineTop, hLineBottom)
    painter.setPen(self._pen)
    painter.drawLines([vLine, hLine])
