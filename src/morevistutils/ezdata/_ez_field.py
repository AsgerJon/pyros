"""EZField provides a descriptor class for the EZData class. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils import monoSpace
from vistutils.fields import AbstractField

from morevistutils.waitaminute import typeMsg


class EZField(AbstractField):
  """EZField provides a descriptor class for the EZData class. """

  def __init__(self, type_: type, defVal: Any = None) -> None:
    AbstractField.__init__(self, )
    self.__value_type__ = None
    self.__default_value__ = None
    if defVal is None:
      if isinstance(type_, type):
        self.__value_type__ = type_
        self.__default_value__ = None
      else:
        self.__default_value__ = type_
        self.__value_type__ = type(type_)
    if isinstance(type_, type):
      if isinstance(defVal, type_):
        self.__default_value__ = defVal
        self.__value_type__ = type_
    if self.__value_type__ is None:
      raise TypeError

  def __prepare_owner__(self, owner: type) -> type:
    """Implementation of the abstract method"""
    return owner

  def __prepare_instance__(self, instance: Any) -> Any:
    """Prepares the instance"""
    pvtName = self._getPrivateName()
    defVal = self.__default_value__
    if defVal is None:
      e = """Instance of '%s' was not created with default value!"""
      raise ValueError(monoSpace(e % self.__class__.__qualname__))
    setattr(instance, pvtName, self.__default_value__)

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    """Getter-function"""
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName, ):
      val = getattr(instance, pvtName)
      if isinstance(val, self.__value_type__):
        return val
      e = typeMsg('val', val, self.__value_type__)
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    self.__prepare_instance__(instance)
    return self.__get__(instance, owner, _recursion=True)

  def __set__(self, instance: Any, value: Any) -> None:
    """Setter-function"""
    pvtName = self._getPrivateName()
    if isinstance(value, self.__value_type__):
      return setattr(instance, pvtName, value)
    e = typeMsg('value', value, self.__value_type__)
    raise TypeError(e)

  def __delete__(self, instance: Any) -> None:
    """Deleter function"""
    delattr(instance, self._getPrivateName())
