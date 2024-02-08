"""PaintWidget provides a baseclass for the painted widgets"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPaintEvent, QPainter
from icecream import ic

from morevistside.paintmelike import AbstractStencil
from morevistside.widgets import BaseWidget

ic.configureOutput(includeContext=True)


class PaintWidget(BaseWidget):
  """PaintWidget provides a baseclass for the painted widgets"""

  __stencil_fields__ = []

  @classmethod
  def _getStencils(cls) -> list[AbstractStencil]:
    """Getter-function for list of stencils"""
    stencils = getattr(cls, '__stencil_fields__', None)
    if stencils is None:
      raise AttributeError('__stencil_fields__')
    return stencils

  def paintEvent(self, event: QPaintEvent) -> None:
    """Creates a painter and paints each stencil"""
    painter = QPainter()
    painter.begin(self)
    for stencil in self._getStencils():
      stencil.paintMeLike(painter, event)
    painter.end()
