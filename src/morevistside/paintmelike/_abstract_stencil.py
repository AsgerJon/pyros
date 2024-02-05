"""The AbstractStencil provides the abstract baseclass for paint
actions. These are descriptor classes owned by widgets. The getter returns
a callable based on the widget. This callable takes as argument a QPainter
object that must be defined on the widget and be active. The callable then
defines how the painter should paint the widget."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QPaintEvent, QPen, QColor, QBrush
from PySide6.QtWidgets import QWidget
from vistutils.fields import AbstractField

from morevistside import shibokinator
from morevistutils.waitaminute import typeMsg


class AbstractStencil(AbstractField):
  """The AbstractStencil provides the abstract baseclass for paint
  actions. These are descriptor classes owned by widgets. The getter returns
  a callable based on the widget. This callable takes as argument a QPainter
  object that must be defined on the widget and be active. The callable then
  defines how the painter should paint the widget."""

  __attribute_name__ = '__stencil_fields__'

  def __prepare_owner__(self, owner: type) -> type:
    """Ensures that owner is a QWidget or a subclass of QWidget"""
    if not shibokinator(owner)[0]:
      e = """Only subclasses of 'QWidget' can own stencils."""
      raise TypeError(e)
    existing = getattr(owner, self.__attribute_name__, [])
    setattr(owner, self.__attribute_name__, [*existing, self])
    return owner

  def __getattribute__(self, key: str) -> Any:
    """Implementation of this method was done by a seasoned professional
    in safe conditions. Do not try this at home!"""
    accessorNames = ['__get__', '__set__', '__delete__']
    for name in accessorNames:
      if key == name:
        object.__getattribute__(self, key)()
        break
    return object.__getattribute__(self, key)

  def __init__(self, *args, **kwargs) -> None:
    AbstractField.__init__(self, *args, **kwargs)
    self.__decorated_getter__ = None
    self.__decorated_setter__ = None
    self.__decorated_deleter__ = None

  def __get__(self, instance: Any, owner: type) -> Any:
    """Returns the value using the method defined by the decorator.
    Subclasses may reimplement this method to provide alternate accessor
    functionality. """
    if self.__decorated_getter__ is None:
      raise RuntimeError
    if callable(self.__decorated_getter__):
      return self.__decorated_getter__(self, instance, owner)

  def __set__(self, instance: Any, value: Any) -> None:
    """Returns the value using the method defined by the decorator.
    Subclasses may reimplement this method to provide alternate accessor
    functionality. """
    if self.__decorated_setter__ is None:
      raise RuntimeError
    if callable(self.__decorated_setter__):
      return self.__decorated_setter__(self, instance, value)

  def __delete__(self, instance: Any, ) -> None:
    """Returns the value using the method defined by the decorator.
    Subclasses may reimplement this method to provide alternate accessor
    functionality. """
    if self.__decorated_deleter__ is None:
      raise RuntimeError
    if callable(self.__decorated_deleter__):
      return self.__decorated_deleter__(self, instance, )

  def GET(self, callMeMaybe: Callable) -> Callable:
    """Decorator defining the callable as the getter function. """
    self.__decorated_getter__ = callMeMaybe
    return callMeMaybe

  def SET(self, callMeMaybe: Callable) -> Callable:
    """Decorator defining the callable as the setter function. """
    self.__decorated_setter__ = callMeMaybe
    return callMeMaybe

  def DEL(self, callMeMaybe: Callable) -> Callable:
    """Decorator defining the callable as the deleter function. """
    self.__decorated_deleter__ = callMeMaybe
    return callMeMaybe

  def __notify_get__(self, ) -> None:
    """Triggered by accessing __get__ attribute."""

  def __notify_set__(self, ) -> None:
    """Triggered by accessing __set__ attribute."""

  def __notify_del__(self, ) -> None:
    """Triggered by accessing __delete__ attribute."""

  def paintMeLike(self, painter: QPainter, event: QPaintEvent) -> None:
    """This method defines how this descriptor should contribute to the
    painting of the owning widget instance. The base class implements type
    guarding by raising an error if the paint device of the painter is not
    an instance of the field owner. Subclasses reimplementing this method
    may conveniently invoke the super call before their own
    implementations."""
    owner = self._getFieldOwner()
    device = painter.device()
    if not isinstance(device, owner):
      e = typeMsg('device', device, owner)
      raise TypeError(e)

  @staticmethod
  def _getEmptyPen() -> QPen:
    """Getter-function for the empty pen"""
    pen = QPen()
    pen.setStyle(Qt.PenStyle.NoPen)
    pen.setColor(QColor(0, 0, 0, 0, ))
    pen.setWidth(1)
    return pen

  @staticmethod
  def _getEmptyBrush() -> QBrush:
    """Getter-function for the empty brush"""
    brush = QBrush()
    brush.setStyle(Qt.BrushStyle.NoBrush)
    brush.setColor(QColor(0, 0, 0, 0, ))
    return brush
