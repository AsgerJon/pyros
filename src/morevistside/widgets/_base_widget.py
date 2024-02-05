"""BaseWidget is a blank subclass of QWidget."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from PySide6.QtCore import QSize, QSizeF
from PySide6.QtGui import QResizeEvent
from PySide6.QtWidgets import QWidget
from icecream import ic
from vistutils import maybe
from vistutils.fields import Field

from morevistside import parseParent
from morevistutils.fields import TypedField
from morevistutils.waitaminute import typeMsg


class BaseWidget(QWidget):
  """BaseWidget is a blank subclass of QWidget."""

  minWidth = Field()
  minHeight = Field()
  minSize = Field()

  @minWidth.GET
  def _getMinWidth(self) -> int:
    """Getter-function for minimum width"""
    return self.__minimum_width__

  @minWidth.SET
  def _setMinWidth(self, width: int) -> None:
    """Setter-function for minimum width"""
    self.__minimum_width__ = width
    self._applyMinSize()

  @minHeight.GET
  def _getMinHeight(self) -> int:
    """Getter-function for minimum height"""
    return self.__minimum_height__

  @minHeight.SET
  def _setMinHeight(self, height: int) -> None:
    """Setter-function for minimum height"""
    self.__minimum_height__ = height
    self._applyMinSize()

  @minSize.GET
  def __get_min_size__(self) -> QSize:
    """Getter-function for minimum-size"""
    try:
      return self._getMinSize()
    except NotImplementedError:
      return QSize(self.minWidth, self.minHeight)

  def _getMinSize(self) -> QSize:
    """Getter-function available for subclasses"""
    raise NotImplementedError

  @minSize.SET
  def _setMinSize(self, *args) -> None:
    """Setter-function for minimum size"""
    badArgs = []
    for arg in args:
      if isinstance(arg, QSizeF):
        return self._setMinSize(arg.toSize())
      if isinstance(arg, QSize):
        self.__minimum_width__ = arg.width()
        self.__minimum_height__ = arg.height()
        return self._applyMinSize()
      badArgs.append(typeMsg('newSize', arg, QSize))
    e = '\n'.join(badArgs)
    raise TypeError(e)

  def _applyMinSize(self, ) -> None:
    """Applies the values in the size fields to the actual size"""
    oldSize = self.size()
    newWidth = max(oldSize.width(), self.__minimum_width__)
    newHeight = max(oldSize.height(), self.__minimum_height__)
    newSize = QSize(newWidth, newHeight)
    event = QResizeEvent(newSize, oldSize)
    return QWidget.resizeEvent(self, event)

  def resizeEvent(self, event: QResizeEvent) -> None:
    """Implementation of resizing"""
    oldSize = self.size()
    newWidth = min(event.size().width(), self.minWidth)
    newHeight = min(event.size().height(), self.minHeight)
    newSize = QSize(newWidth, newHeight)
    newEvent = QResizeEvent(newSize, oldSize)
    QWidget.resizeEvent(self, newEvent)

  def __init__(self, *args, **kwargs) -> None:
    widthArg = None
    heightArg = None
    otherArgs = []
    for arg in args:
      if isinstance(arg, int) and widthArg is None:
        widthArg = arg
      elif isinstance(arg, int) and heightArg is None:
        heightArg = arg
      else:
        otherArgs.append(arg)
    widthDefault = int(os.environ.get('DEFAULT_WIDGET_WIDTH', 24))
    heightDefault = int(os.environ.get('DEFAULT_WIDGET_HEIGHT', 24))

    self.__minimum_width__ = maybe(widthArg, widthDefault)
    self.__minimum_height__ = maybe(heightArg, heightDefault)

    parent = parseParent(*otherArgs)
    QWidget.__init__(self, parent)
    self._applyMinSize()
