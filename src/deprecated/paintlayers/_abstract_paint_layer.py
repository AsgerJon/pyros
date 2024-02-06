"""AbstractPainterPath provides an abstract baseclass for painting paths."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import warnings
from abc import abstractmethod
from typing import Optional, Never, Callable

from icecream import ic
from vistutils import monoSpace
from vistutils.fields import AbstractField, Field
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QColor, QPainterPath, QPaintDevice
from PySide6.QtGui import QPen, QFont, QBrush
from PySide6.QtGui import QPainter
from vistutils.waitaminute import typeMsg

from morevistside import ShibokenType
from morevistutils.waitaminute import WaitForIt

ic.configureOutput(includeContext=True)


class AbstractPaintLayer(AbstractField):
  """AbstractPainterPath provides an abstract baseclass for painting
  paths."""

  __default_brush_color__ = (225, 225, 225, 255)
  __default_brush_style__ = Qt.BrushStyle.SolidPattern

  @classmethod
  def getDefaultBrushColor(cls) -> QColor:
    """Getter-function for defined default color as QColor"""
    return QColor(*cls.__default_brush_color__, )

  @classmethod
  def getDefaultBrushStyle(cls, ) -> Qt.BrushStyle:
    """Getter-function fro defined default style"""
    return cls.__default_brush_style__

  @classmethod
  def _parseBrush(cls, *args, ) -> QBrush:
    """Parses positional arguments to instance of QBrush"""
    brush = None
    color = None
    style = None
    for arg in args:
      if isinstance(arg, QBrush) and brush is None:
        brush = arg
        color = brush.color()
        style = brush.style()
        break
      if isinstance(arg, QColor) and color is None:
        color = arg
      if isinstance(arg, Qt.BrushStyle) and style is None:
        style = arg
    if color is None:
      color = cls.getDefaultBrushColor()
    if style is None:
      style = cls.getDefaultBrushStyle()
    if brush is None:
      brush = QBrush()
      brush.setStyle(style)
      brush.setColor(color)
    if isinstance(brush, QBrush):
      return brush
    e = typeMsg('brush', brush, QBrush)
    raise TypeError(e)

  @classmethod
  def _parseFont(cls, *args) -> QFont:
    """Parses positional arguments to font."""
    warnings.warn(WaitForIt('Placeholder used for QFont'))
    font = QFont()
    font.setFamily('Courier')
    font.setPointSize(16)
    return font

  @classmethod
  def _parsePen(cls, *args, ) -> QPen:
    """Parses positional arguments to QPen"""
    warnings.warn(WaitForIt('Placeholder used for QPen'))
    pen = QPen()
    pen.setStyle(Qt.PenStyle.SolidLine)
    pen.setWidth(1)
    pen.setColor(QColor(0, 0, 0, 255))
    return pen

  @classmethod
  def _parseTextFlags(cls, *args) -> Qt.TextFlag:
    """Parses positional arguments to Qt.TextFlag"""
    warnings.warn(WaitForIt('Placeholder used for text flags'))
    return Qt.TextFlag.TextWordWrap

  _pvtName = Field()

  @_pvtName.GET
  def _getPvtName(self) -> str:
    """Uses parent method"""
    return self._getPrivateName()

  def __init__(self, *args, **kwargs) -> None:
    AbstractField.__init__(self, *args, **kwargs)

  def _findValue(self, instance: QPaintDevice) -> Callable:
    """Locates an instance at given instance"""
    if hasattr(instance, self._pvtName):
      painterHandle = getattr(instance, self._pvtName)
      if callable(painterHandle):
        return painterHandle
      e = typeMsg('painterHandle', painterHandle, Callable)
      raise TypeError(e)

  def __get__(self,
              instance: QPaintDevice,
              owner: ShibokenType,
              **kwargs) -> Callable:
    """Returns a path appropriate for given instance."""
    if instance is None:
      e = """Cannot create path on class: '%s', only on instance hereof!"""
      raise TypeError(monoSpace(e % owner.__qualname__))
    existing = self._findValue(instance, )
    if existing is None:
      setattr(instance, self._pvtName, self.paintMeLike)
      return self.__get__(instance, owner, _recursion=True, **kwargs)
    return existing

  def __set__(self, *_) -> Never:
    """Illegal accessor!"""
    e = """Cannot call setter on immutable painter path!"""
    raise TypeError(e)

  def __delete__(self, *_) -> Never:
    """Illegal accessor!"""
    e = """Cannot call deleter on immutable painter path!"""
    raise TypeError(e)

  @abstractmethod
  def paintMeLike(self, painter: QPainter) -> None:
    """This method is invoked during the paint event allowing subclasses
    to define how they are painted on to the paint device."""

  def __prepare_owner__(self, owner: ShibokenType) -> ShibokenType:
    """Performs runtime type check"""
    if not isinstance(owner, ShibokenType):
      e = typeMsg('owner', owner, ShibokenType)
      raise TypeError(e)
    owner.layers.append(self)
    return owner
