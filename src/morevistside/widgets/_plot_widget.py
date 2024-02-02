"""PlotWidget provides a visual representation of a numerically defined
field."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QColor

from morevistside.paintlayers import AxesLayer, SolidLayer
from morevistside.widgets import AbstractPaintWidget


class PlotWidget(AbstractPaintWidget):
  """PlotWidget provides a visual representation of a numerically defined
  field."""

  fill = SolidLayer(QColor(127, 255, 0, 255))
  axes = AxesLayer()
