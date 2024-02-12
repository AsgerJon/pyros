"""Alternative super slow implementation of the data roll for use in
comparisons and memes."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class TurtleRoll:
  """Circular buffer implementation using Python list."""

  def __init__(self, capacity: int) -> None:
    self.capacity = capacity
    self.buffer = [None] * capacity
    self.start = 0
    self.end = 0
    self.count = 0

  def append(self, item) -> None:
    """Appends an item to the buffer."""
    if self.count == self.capacity:
      self.start = (self.start + 1) % self.capacity
    else:
      self.count += 1
    self.buffer[self.end] = item
    self.end = (self.end + 1) % self.capacity

  def __iter__(self) -> TurtleRoll:
    """Allows iteration over buffer items."""
    idx = self.start
    for _ in range(self.count):
      yield self.buffer[idx]
      idx = (idx + 1) % self.capacity
