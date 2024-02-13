"""DataRoll leverages numba to achieve significant speed improvements
compared to native python structures."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Generic, TypeVar, Any
from warnings import warn

import numpy as np
from numpy.typing import NDArray, DTypeLike

T = TypeVar('T')


class DataRoll(Generic[T]):
  """Implements a high-performance circular buffer using Numba.
  
  Attributes:
    __buffer_size__ (int): The fixed size of the buffer.
    __buffer_array__ (NDArray): The array storing buffer elements.
    __begin_index__ (int): The starting index, managed by Numba functions.
  """

  def __init__(self,
               size: int,
               fillValue: Any = None, ) -> None:
    """Initializes the circular buffer with a given size and dtype."""
    self.__data_type__ = np.float64
    self.__buffer_size__ = size
    self.__default_value__ = 0 if fillValue is None else fillValue
    self.__buffer_array__ = np.ones(
      size, dtype=self.__data_type__) * self.__default_value__
    self.__begin_index__ = 0
    self.__iter_array__ = np.zeros(size, dtype=self.__data_type__)
    self.__iter_index__ = 0

  def append(self, item: T) -> None:
    """Appends an item to the buffer, overwriting the oldest if full."""
    self.__buffer_array__[self.__begin_index__] = item
    self.__begin_index__ += 1
    self.__begin_index__ %= self.__buffer_size__

  def getArray(self) -> NDArray:
    """Getter-function for the buffer array."""
    return np.concatenate(
      (self.__buffer_array__[self.__begin_index__:],
       self.__buffer_array__[:self.__begin_index__]))

  def reset(self, ) -> None:
    """Reorders the buffer array"""
    self.__buffer_array__ = self.getReset()

  def getReset(self) -> NDArray:
    """Resets the buffer to its default value."""
    return np.concatenate(
      (self.__buffer_array__[self.__begin_index__:],
       self.__buffer_array__[:self.__begin_index__]))

  def __iter__(self) -> DataRoll:
    """Yields the buffer's current elements."""
    self.__iter_array__ = self.getReset()
    return self

  def __next__(self, ) -> Any:
    """Implements iteration."""
    try:
      self.__iter_index__ += 1
      return self.__iter_array__[self.__iter_index__ - 1]
    except IndexError:
      raise StopIteration

  def __len__(self) -> int:
    """Returns the number of items in the buffer."""
    return self.__buffer_size__

  def __modulus_index__(self, index: int) -> int:
    """Returns the index inside the buffer."""
    while index < 0:
      return self.__modulus_index__(index + len(self))
    while index >= len(self):
      return self.__modulus_index__(index - len(self))
    return index

  def __getitem__(self, index: Any) -> T:
    """Item retrieval."""
    if isinstance(index, int):
      index = self.__modulus_index__(index)
      return self.__buffer_array__[index]
    if isinstance(index, slice):
      start = self.__modulus_index__(index.start)
      stop = self.__modulus_index__(index.stop)
      if index.step is not None:
        w = """Step is not supported in DataRoll slicing, and will be 
        ignored."""
        warn(w)
      return self.__buffer_array__[start:stop]
    if isinstance(index, str):
      if index.lower()[:3] == 'min':
        return min(self.__buffer_array__)
      if index.lower()[:3] == 'max':
        return max(self.__buffer_array__)
      if index.lower()[:3] == 'sum':
        return sum(self.__buffer_array__)

  def __sub__(self, other) -> DataRoll:
    """Subtracts the buffer from another array."""
    if isinstance(other, (int, float)):
      out = DataRoll(self.__buffer_size__, self.__default_value__)
      out.__buffer_array__ = self.__buffer_array__ - float(other)
      return out
    return NotImplemented

  def __add__(self, other) -> DataRoll:
    """Adds the buffer to another array."""
    if isinstance(other, (int, float)):
      out = DataRoll(self.__buffer_size__, self.__default_value__)
      out.__buffer_array__ = self.__buffer_array__ + float(other)
      return out
    return NotImplemented

  def __mul__(self, other) -> DataRoll:
    """Multiplies the buffer with another array."""
    if isinstance(other, (int, float)):
      out = DataRoll(self.__buffer_size__, self.__default_value__)
      out.__buffer_array__ = np.ones(self.__buffer_size__) * float(other)
      return out
    return NotImplemented
