"""DataRoll leverages numba to achieve significant speed improvements
compared to native python structures."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Generic, TypeVar, Tuple
import numpy as np
from numba import njit
from numpy.typing import NDArray, DTypeLike

T = TypeVar('T')


class DataRoll(Generic[T]):
  """Implements a high-performance circular buffer using Numba.
  
  Attributes:
    size (int): The fixed size of the buffer.
    buffer (NDArray): The array storing buffer elements.
    start (int): The starting index, managed by Numba functions.
    end (int): The ending index, managed by Numba functions.
    isFull (bool): Flag to check if buffer is full, managed by Numba.
    dtype (DTypeLike): Data type of the buffer elements.
  """

  def __init__(self, size: int, dtype: DTypeLike) -> None:
    """Initializes the circular buffer with a given size and dtype."""
    self.size: int = size
    self.buffer: NDArray[T] = np.empty(size, dtype=dtype)
    self.start: int = 0
    self.end: int = 0
    self.isFull: bool = False
    self.dtype: DTypeLike = dtype

  def append(self, item: T) -> None:
    """Appends an item to the buffer, overwriting the oldest if full."""
    self.start, self.end, self.isFull = append_numba(
      self.buffer, self.start, self.end, self.isFull, item, self.size
    )

  def __iter__(self) -> NDArray[T]:
    """Yields the buffer's current elements."""
    return iter_numba(self.buffer,
                      self.start,
                      self.end,
                      self.isFull,
                      self.size)


@njit
def append_numba(buffer: NDArray[T], start: int, end: int, isFull: bool,
                 item: T, size: int) -> Tuple[int, int, bool]:
  """Numba-compiled function to append items to the buffer."""
  if isFull:
    start = (start + 1) % size
  buffer[end] = item
  end = (end + 1) % size
  if end == start:
    isFull = True
  else:
    isFull = False
  return start, end, isFull


@njit
def iter_numba(buffer: NDArray[T], start: int, end: int, isFull: bool,
               size: int) -> NDArray[T]:
  """Numba-compiled function to get a snapshot of the buffer for
  iteration."""
  if isFull or start <= end:
    return buffer[start:end]
  else:  # Handle wrap-around
    return np.concatenate((buffer[start:], buffer[:end]))
