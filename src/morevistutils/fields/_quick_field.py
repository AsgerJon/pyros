"""QuickField class for quick and easy field creation."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils.fields import AbstractField


class QuickField(AbstractField):
  """QuickField class for quick and easy field creation."""

  def __prepare_owner__(self, owner: type) -> type:
    """Prepare the owner class for the field."""
    return owner

  def __init__(self, *args) -> None:
    AbstractField.__init__(self)
    self.__default_value__ = None
    self.__data_type__ = None
    if not args:
      raise ValueError('Missing arguments')
    if len(args) == 1:
      if isinstance(args[0], type):
        self.__data_type__ = args[0]
      else:
        self.__data_type__ = type(args[0])
        self.__default_value__ = args[0]
    elif len(args) > 1:
      if isinstance(args[0], type):
        self.__data_type__ = args[0]
        defVal = args[1]
      elif isinstance(args[1], type):
        self.__data_type__ = args[1]
        defVal = args[0]
      else:
        raise TypeError
      if not isinstance(defVal, self.__data_type__):
        raise TypeError
      self.__default_value__ = defVal
    if self.__data_type__ is None:
      raise ValueError

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    """Getter-function for the field"""
    if instance is None:
      if self.__default_value__ is None:
        raise AttributeError
      return self.__default_value__
    if hasattr(instance, self._getPrivateName()):
      return getattr(instance, self._getPrivateName())
    if kwargs.get('_recursion', False):
      raise RecursionError
    setattr(instance, self._getPrivateName(), self.__default_value__)
    return self.__get__(instance, owner, _recursion=True)

  def __set__(self, instance: Any, value: Any, **kwargs) -> None:
    """Setter-function for the field"""
    if not isinstance(value, self.__data_type__):
      raise TypeError
    setattr(instance, self._getPrivateName(), value)

  def __delete__(self, instance: Any) -> None:
    """Deleter-function for the field"""
    delattr(instance, self._getPrivateName())
