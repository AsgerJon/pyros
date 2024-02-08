"""PlotWidget provides a visual representation of a numerically defined
field."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QColor

from morevistside.paintmelike import SolidBackground, DataPlot
from morevistside.widgets import PaintWidget


class PlotWidget(PaintWidget):
  """PlotWidget provides a visual representation of a numerically defined
  field."""

  fill = SolidBackground(QColor(255, 255, 0, 255))
  dataPlot = DataPlot()

  def __init__(self, *args, **kwargs) -> None:
    PaintWidget.__init__(self, *args, **kwargs)
