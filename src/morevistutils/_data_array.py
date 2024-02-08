"""DataArray subclasses ndarray from numpy"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any

from PySide6.QtCore import QPointF
from icecream import ic
from numpy import ndarray, float64, full, nan

ic.configureOutput(includeContext=True)


class DataArray:
  """DataArray subclasses ndarray from numpy"""

  def __init__(self, n: int, *args, **kwargs) -> None:
    self.__inner_length__ = n
    self.__inner_values__ = full((n,), nan)
    self.__inner_times__ = full((n,), nan)
    self.__outer_values__ = None
    self.__outer_times__ = None
    self.__current_index__ = 0

  def callBack(self, data: Any) -> Any:
    """Callback implementation """
    epochTime = time.time()
    self.__inner_times__[self.__current_index__] = time.time()
    self.__inner_values__[self.__current_index__] = data.data
    self.__current_index__ = (self.__current_index__ + 1) % len(self)

  def __len__(self, ) -> int:
    """Length is understood as the length of the vectors"""
    return self.__inner_length__

  def __iter__(self, ) -> DataArray:
    """Implementation of iteration"""
    self.__outer_times__ = self.__inner_times__.tolist()
    self.__outer_values__ = self.__inner_values__.tolist()
    return self

  def __next__(self, ) -> tuple[float, float]:
    """Returns the time-value pair as an instance of QPointF"""
    try:
      t, x = self.__outer_times__.pop(0), self.__outer_values__.pop(0)
    except IndexError:
      raise StopIteration
    return t, x

  def getScaledTimes(self, scaleT: float = None) -> ndarray:
    """Getter-function for the scaled time"""
    if scaleT is None:
      return self.__inner_times__.copy()
    return scaleT * self.__inner_times__.copy()

  def getScaledValues(self, scaleX: float = None) -> ndarray:
    """Getter-function for the scaled values"""
    if scaleX is None:
      return self.__inner_values__.copy()
    return scaleX * self.__inner_values__.copy()
