"""DiagonalPath is a subclass of ParameterPath that draws two diagonal
line across the canvas. They take no additional parameter."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QPointF

from morevistside.paintpaths import ParameterPath, CanvasPath


class DiagonalPath(ParameterPath):
  """DiagonalPath is a subclass of ParameterPath that draws two diagonal
  line across the canvas. They take no additional parameter."""

  def __init__(self, *paths, **kwargs) -> None:
    ParameterPath.__init__(self, *paths, **kwargs)

  def initialize(self, canvas: CanvasPath, *parameters) -> None:
    """Draws from top left to bottom right, and then from top right to
    bottom left."""
    self.clear()
    self.moveTo(QPointF(0, 0, ))
    self.lineTo(QPointF(canvas.width, canvas.height))
    self.moveTo(QPointF(canvas.width, 0))
    self.lineTo(QPointF(0, canvas.height))
