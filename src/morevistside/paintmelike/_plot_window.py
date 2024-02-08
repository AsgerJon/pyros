"""PlotWindow provides a descriptor for the real values of the boundaries,
not the pixel values. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from morevistutils.ezdata import EZData


class PlotWindow(EZData):
  """PlotWindow provides a descriptor for the real values of the boundaries,
  not the pixel values. """

  tMin: int
  tMax: int
  xMin: int
  xMax: int
