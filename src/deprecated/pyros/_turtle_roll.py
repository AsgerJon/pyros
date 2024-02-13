"""Alternative super slow implementation of the data roll for use in
comparisons and memes."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any


class TurtleRoll:
  """Circular buffer implementation using Python list."""

  def __init__(self, capacity: int) -> None:
    self.__buffer_capacity__ = capacity
    self.__buffer_contents__ = [None] * capacity
    self.__iter_contents__ = None

  def append(self, item) -> None:
    """Appends an item to the buffer."""
    self.__buffer_contents__.append(item)
    while len(self.__buffer_contents__) > self.__buffer_capacity__:
      self.__buffer_contents__.pop(0)

  def __iter__(self) -> TurtleRoll:
    """Allows iteration over buffer items."""
    self.__iter_contents__ = [i for i in self.__buffer_contents__]
    while len(self.__iter_contents__) < self.__buffer_capacity__:
      self.__iter_contents__.insert(0, None)
    return self

  def __next__(self, ) -> Any:
    """Implementing iteration"""
    try:
      return self.__iter_contents__.pop(0)
    except IndexError:
      raise StopIteration

  def __getitem__(self, index: int) -> Any:
    """Item retrieval"""
    index = self.__roll_index__(index)
    return list.__getitem__(self.__buffer_contents__, index)

  def __setitem__(self, index: int, value: Any) -> Any:
    """Item retrieval"""
    index = self.__roll_index__(index)
    return list.__setitem__(self.__buffer_contents__, index, value)

  def __len__(self, ) -> int:
    """Length is taken the number of items in the buffer, even if this
    number is not yet at capacity"""
    return list.__len__(self.__buffer_contents__)

  def __roll_index__(self, index: int) -> int:
    """Returns the index inside the buffer"""
    while index < 0:
      return self.__roll_index__(index + len(self))
    while index >= len(self):
      return self.__roll_index__(index - len(self))
    return index
