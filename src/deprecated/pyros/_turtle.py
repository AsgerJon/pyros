"""Turtle uses circular data structures"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from pyros import DataRoll


class Turtle:
  """Turtle class that uses a circular buffer to store positions."""

  def __init__(self, capacity: int) -> None:
    self._positions = DataRoll(capacity)

  def moveTo(self, position) -> None:
    """Records the turtle's new position."""
    self._positions.append(position)

  def getPositions(self) -> list:
    """Returns all positions stored in the buffer."""
    return [i for i in self._positions]
