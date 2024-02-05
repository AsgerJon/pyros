"""PlotWidget provides a visual representation of a numerically defined
field."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QColor

from morevistside import parseParent
from morevistside.paintlayers import AxesLayer, SolidLayer
from morevistside.widgets import AbstractPaintWidget


class PlotWidget(AbstractPaintWidget):
  """PlotWidget provides a visual representation of a numerically defined
  field."""

  fill = SolidLayer(QColor(127, 255, 0, 255))
  axes = AxesLayer()

  def __init__(self, *args, **kwargs) -> None:
    AbstractPaintWidget.__init__(self, *args, **kwargs)
