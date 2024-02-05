"""AbstractPaintWidget collects all paint layers defined on the class body
and paints them during the paint event. The baseclass does not include any
layers. Subclasses should place layers at the top of the class body."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import sys
from typing import TYPE_CHECKING

from PySide6.QtCore import QSize
from PySide6.QtGui import QPaintEvent, QPainter
from vistutils.fields import Field

from morevistside.widgets import BaseWidget

if TYPE_CHECKING:
  from morevistside.paintlayers import AbstractPaintLayer


class AbstractPaintWidget(BaseWidget):
  """PaintWidget is a subclass of BaseWidget and implements the paint event
  through the use of path fields. """

  __paint_layers__ = None

  layers = Field()

  @classmethod
  def _createLayers(cls, ) -> None:
    """Creator function for paint layers"""
    cls.__paint_layers__ = []

  @classmethod
  @layers.GET
  def _getLayers(cls, **kwargs) -> list[AbstractPaintLayer]:
    """Getter-function for list of layers"""
    if cls.__paint_layers__ is None:
      if kwargs.get('_recursion', None):
        raise RecursionError
      cls._createLayers()
      return cls._getLayers(_recursion=True)
    return cls.__paint_layers__

  def paintEvent(self, event: QPaintEvent) -> None:
    """Implementation of painting. Subclasses should not need to implement
    this method. """
    self._applyMinSize()
    for layer in self.layers:
      painter = QPainter()
      painter.begin(self)

      try:
        layer.paintMeLike(painter)
      except Exception as e:
        print(e)
        return painter.end()
      painter.end()

  def _getMinSize(self) -> QSize:
    """Getter-function available for subclasses"""
    return BaseWidget._getMinSize(self)
