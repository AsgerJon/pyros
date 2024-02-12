"""Space specifies 2d ranges"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable

from morevistutils.ezdata import EZData


class Point(EZData):
  """Point specifies a 2d point"""

  t: float
  x: float

  def __str__(self) -> str:
    """String representation"""
    return '(%.3f, %.3f)' % (self.t, self.x)


class Space(EZData):
  """Space specifies 2d ranges"""

  t0: float
  t1: float
  x0: float
  x1: float

  def __matmul__(self, other: Space) -> Callable:
    """The matmul operator creates a callable from self to other"""

    def mapTo(point: Point) -> Point:
      """Receives a point in this space and returns the point mapped to
      other space. """

      dt = (point.t - self.t0) / (self.t1 - self.t0)
      dx = (point.x - self.x0) / (self.x1 - self.x0)
      t = (other.t1 - other.t0) * dt + other.t0
      x = (other.x1 - other.x0) * dx + other.x0
      return Point(t, x)

    return mapTo

  def __str__(self, ) -> str:
    """String representation"""
    t0, t1, x0, x1 = self.t0, self.t1, self.x0, self.x1
    return '(%.3f -> %.3f, %.3f -> %.3f)' % (t0, t1, x0, x1)
