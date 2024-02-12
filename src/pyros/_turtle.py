"""Turtle uses circular data structures"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class Turtle:
  """Turtle class that uses a circular buffer to store positions."""

  def __init__(self, capacity: int) -> None:
    self.positions = CircularBuffer(capacity)

  def move(self, position) -> None:
    """Records the turtle's new position."""
    self.positions.append(position)

  def get_positions(self):
    """Returns all positions stored in the buffer."""
    return list(self.positions)
