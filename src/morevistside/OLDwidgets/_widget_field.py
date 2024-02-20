"""WidgetField provides deferred descriptor access to instances of the
given widget type. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Callable, Optional, Any, Never

from PySide6.QtGui import QFont
from PySide6.QtWidgets import QWidget
from icecream import ic
from vistutils.fields import AbstractField
from vistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class WidgetField(AbstractField):
  """WidgetField provides deferred descriptor access to instances of the
  given widget type. """

  @staticmethod
  def _validateWidgetType(widgetType: type) -> type:
    """Validator for the given widget type"""
    if widgetType is QWidget:
      return widgetType
    if issubclass(widgetType, QWidget):
      return widgetType
    e = """Expected subclass of QWidget, but received: '%s'"""
    raise TypeError(e % widgetType.__qualname__)

  def __init__(self, widgetType: type, *args, **kwargs) -> None:
    AbstractField.__init__(self, *args, **kwargs)
    self.__widget_type__ = self._validateWidgetType(widgetType)
    self.__decorated_creator__ = None
    self.__positional_arguments__ = [arg for arg in args]
    self.__keyword_arguments__ = {k: v for (k, v) in kwargs.items()}

  def __prepare_owner__(self, owner: type) -> type:
    """Reimplementation of abstract method"""
    return owner

  def _getWidgetType(self) -> type:
    """Getter-function for widget type"""
    return self.__widget_type__

  def _getDecoratedCreator(self) -> Optional[Callable]:
    """Getter-function for decorated creator if present"""
    return self.__decorated_creator__

  def _setDecoratedCreator(self, callMeMaybe: Callable) -> None:
    """Run-once setter-function for the decorated creator-function"""
    if self.__decorated_creator__ is not None:
      raise TypeError
    self.__decorated_creator__ = callMeMaybe

  def getCreator(self) -> Callable:
    """Getter-function for the creator function. """
    decoratedCreator = self._getDecoratedCreator()
    if decoratedCreator is not None:
      if callable(decoratedCreator):
        return decoratedCreator
      e = typeMsg('decoratedCreator', decoratedCreator, Callable)
      raise TypeError(e)

    def newWidget(*args, **kwargs) -> QWidget:
      """This inferred method calls the widget type directly"""
      widgetType = self._getWidgetType()
      return widgetType(*args, **kwargs)

    return newWidget

  def CREATE(self, callMeMaybe: Callable) -> Callable:
    """Decorates the callable as the creator function for this instance.
    If defined, this takes precedence. """
    if self.__decorated_creator__ is not None:
      raise AttributeError
    self.__decorated_creator__ = callMeMaybe
    return callMeMaybe

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    if instance is None or instance is owner:
      raise NotImplementedError
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      return getattr(instance, pvtName)
    if kwargs.get('_recursion', False):
      raise RecursionError
    creator = self.getCreator()
    widget = creator(instance, *self.__positional_arguments__,
                     **self.__keyword_arguments__)
    setattr(instance, pvtName, widget)
    return self.__get__(instance, owner, _recursion=True, **kwargs)

  def __set__(self, instance: Any, value: Any) -> None:
    """Setter behaviour should be customized to suit the inner widget
    type. If value is of type str and widget type implements setText,
    this method should call setText(widget, value)."""
    widget = self.__get__(instance, instance.__class__)
    if isinstance(value, str):
      if hasattr(widget, 'display'):
        return widget.display(value)
      if hasattr(widget, 'setText'):
        return widget.setText(value)
    if isinstance(value, QFont):
      if hasattr(widget, 'setFont'):
        return widget.setFont(value)

  def __delete__(self, instance) -> Never:
    """Illegal deleter function"""
    raise TypeError('Protected variable')
