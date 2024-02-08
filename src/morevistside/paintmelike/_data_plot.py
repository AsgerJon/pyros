"""DataPlot receives and displays data"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Never

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QPainter, QPaintEvent, QPen, QColor
from icecream import ic
from vistutils import monoSpace

from morevistside.paintmelike import AbstractStencil, PlotWindow
from morevistutils import DataArray
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class DataPlot(AbstractStencil):
  """DataPlot receives and displays data"""

  __default_length__ = 64

  def __get_array_name__(self) -> str:
    """Getter-function for the data array name"""
    return '__%s_array__' % self._getFieldName()

  def __get_window_name__(self) -> str:
    """Getter-function for the value view name"""
    return '__%s_view__' % self._getFieldName()

  def __init__(self, *args, **kwargs) -> None:
    AbstractStencil.__init__(self, *args, **kwargs)
    self._arrayLength = None
    for arg in args:
      if isinstance(arg, int) and self._arrayLength is None:
        self._arrayLength = arg
    if self._arrayLength is None:
      self._arrayLength = self.__default_length__

  def __prepare_owner__(self, owner: type) -> type:
    """Implementation of abstract method"""
    return owner

  def __prepare_instance__(self, instance: Any) -> Any:
    """Receives an instance of owner class and places a data array
    instance on it. """
    arrayName = self.__get_array_name__()
    windowName = self.__get_window_name__()
    if not hasattr(instance, arrayName):
      array = DataArray(self._arrayLength)
      setattr(instance, arrayName, array)
    if not hasattr(instance, windowName):
      window = PlotWindow(0, 0, 1, 1)
      setattr(instance, windowName, window)
    return instance

  def __get__(self, instance: Any, owner: type, **kwargs) -> DataArray:
    """Getter-function for data array at instance"""
    pvtName = self.__get_array_name__()
    if instance is None:
      e = """Instances of '%s' has the '%s' attribute, but '%s' itself 
      does not!"""
      clsName = owner.__qualname__
      raise AttributeError(monoSpace(e % (clsName, pvtName, clsName,)))
    if hasattr(instance, pvtName):
      return getattr(instance, pvtName)
    if kwargs.get('_recursion', False):
      raise RecursionError
    self.__prepare_instance__(instance)
    return self.__get__(instance, owner, _recursion=True)

  def __set__(self, *_) -> Never:
    """Illegal setter function"""
    e = """DataArray descriptors do not support setting!"""
    raise TypeError(e)

  def __delete__(self, *_) -> Never:
    """Illegal deleter function"""
    e = """DataArray descriptors do not support deleting!"""
    raise TypeError(e)

  def __get_plot_window__(self, instance: Any) -> Any:
    """Getter-function for the plot window"""
    windowName = self.__get_window_name__()
    if hasattr(instance, windowName):
      out = getattr(instance, windowName)
      if isinstance(out, PlotWindow):
        return out
      e = typeMsg('out', out, PlotWindow)
      raise TypeError(e)
    e = """Unable to find attribute named: '%s'!"""
    raise AttributeError(monoSpace(e % windowName))

  @staticmethod
  def _getPointPen() -> QPen:
    """Getter-function for the point pen"""
    pen = QPen()
    pen.setStyle(Qt.PenStyle.SolidLine)
    pen.setColor(QColor(0, 0, 0, 255))
    pen.setWidth(1)
    return pen

  def paintMeLike(self, painter: QPainter, event: QPaintEvent) -> None:
    ic(painter)
    instance = painter.device()
    owner = self._getFieldOwner()
    dataArray = self.__get__(instance, owner)
    valueWindow = self.__get_plot_window__(instance)
    valueWidth = valueWindow.xMax - valueWindow.xMin
    valueHeight = valueWindow.yMax - valueWindow.yMin
    pixelView = painter.viewport()
    pixelWidth = pixelView.width()
    pixelHeight = pixelView.height()
    scaleT = pixelWidth / valueWidth
    scaleX = pixelHeight / valueHeight
    scaledTimes = dataArray.getScaledTimes(scaleT)
    scaledValues = dataArray.getScaledValues(scaleX)
    points = []
    for (t, x) in zip(scaledTimes, scaledValues):
      points.append(QPointF(t, x))
    painter.setBrush(self._getEmptyBrush())
    painter.setPen(self._getPointPen())
    painter.drawPoints(points)
