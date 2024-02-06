"""SolidLayer subclasses AbstractPaintLayer and provides a filled
background"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPainter
from icecream import ic

from deprecated.paintlayers import AbstractPaintLayer

ic.configureOutput(includeContext=True)


class SolidLayer(AbstractPaintLayer):
  """SolidLayer subclasses AbstractPaintLayer and provides a filled
  background"""

  def __init__(self, *args, **kwargs) -> None:
    AbstractPaintLayer.__init__(self, *args, **kwargs)
    self._brush = self._parseBrush(*args)

  def paintMeLike(self, painter: QPainter) -> None:
    """Before the painter draws this path, this method is called on the
    painter. This allows subclasses to make changes to the painter before
    the 'drawRect' is called."""
    painter.setBrush(self._brush)
    viewRect = painter.viewport()
    painter.drawRect(viewRect)
