"""DataArray subclasses ndarray from numpy"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any

import numpy as np
from icecream import ic
from numpy import ndarray, full, nan, isnan

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
    self.__min_time__ = None

  def append(self, value: float) -> Any:
    """Callback implementation """
    epochTime = time.time()
    self.__inner_times__[self.__current_index__] = time.time()
    self.__inner_values__[self.__current_index__] = value
    self.__current_index__ = (self.__current_index__ + 1) % len(self)

  def __len__(self, ) -> int:
    """Length is understood as the length of the vectors"""
    return self.__inner_length__

  def __iter__(self, ) -> DataArray:
    """Implementation of iteration"""
    timeIndices = ~isnan(self.__inner_times__)
    valueIndices = ~isnan(self.__inner_values__)
    indices = timeIndices * valueIndices
    self.__outer_times__ = self.__inner_times__[indices].tolist()
    try:
      self.__min_time__ = min([i for i in self.__outer_times__ if i == i])
    except ValueError as valueError:
      self.__outer_times__ = []
      self.__outer_values__ = []
      return self
    self.__outer_values__ = self.__inner_values__[indices].tolist()
    return self

  def __next__(self, ) -> tuple[float, float]:
    """Returns the time-value pair as an instance of QPointF"""
    try:
      t, x = self.__outer_times__.pop(0), self.__outer_values__.pop(0)
    except IndexError:
      raise StopIteration
    return t - self.__min_time__, x

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

  def getMinT(self) -> float:
    """Getter-function for minimum time"""
    times = [i for i in self.__inner_times__.tolist() if i == i]
    if times:
      return min(times)
    return 1337

  def getMaxT(self) -> float:
    """Getter-function for maximum time"""
    times = [i for i in self.__inner_times__.tolist() if i == i]
    if times:
      return max(times)
    return 69420

  def getMinX(self) -> float:
    """Getter-function for minimum x"""
    values = [i for i in self.__inner_values__.tolist() if i == i]
    if values:
      return min(values)
    return -1

  def getMaxX(self) -> float:
    """Getter-function for maximum x"""
    values = [i for i in self.__inner_values__.tolist() if i == i]
    if values:
      return max(values)
    return 1

  def getTimes(self) -> list:
    """Getter-function for list of times"""
