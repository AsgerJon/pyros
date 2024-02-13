"""DataArray exposes collected rospy data to the Qt framework. This allows
the data collection from rospy to operate at different rates than the
painting updates on the Qt framework."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

import numpy as np
from PySide6.QtCore import QRectF, QPointF

from pyros import DataRoll

NDArray = type(np.ones((1,)))


class DataArray:
  """DataArray exposes collected rospy data to the Qt framework. This allows
  the data collection from rospy to operate at different rates than the
  painting updates on the Qt framework."""

  def __init__(self, *args, **kwargs) -> None:
    self.__buffer_size__ = 32
    self.__invocation_time__ = time.time()
    self.__time_array__ = DataRoll(self.__buffer_size__,
                                   self.__invocation_time__)
    self.__value_array__ = DataRoll(self.__buffer_size__, 0)

  def callback(self, value: float) -> None:
    """Callback receiving data. This should be disconnected from the paint
    events."""
    now = time.time()
    self.__time_array__.append(now)
    self.__value_array__.append(value if value == value else 0)

  def getTimes(self) -> NDArray:
    """Getter-function for the list of times"""
    return self.__time_array__.getArray()

  def getValues(self) -> NDArray:
    """Getter-function for the list of values"""
    return self.__value_array__.getArray()

  def getPoints(self, pixelSpace: QRectF) -> list:
    """Getter-function for the points"""
    T, X = self.getTimes(), self.getValues()
    t0, t1, x0, x1 = np.min(T), np.max(T), np.min(X), np.max(X)
    x0, x1 =
    if (t0 - t1) ** 2 < 1e-08:
      return []  # Instead of raising ZeroDivisionError
    times = (T - t0) / (t1 - t0) * pixelSpace.width()
    values = (X - x0) / (x1 - x0) * pixelSpace.height()
    return [QPointF(t, x) for (t, x) in zip(times, values)]
