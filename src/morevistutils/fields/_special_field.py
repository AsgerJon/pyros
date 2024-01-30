"""SpecialField provides a descriptor class pointing to instances of
custom classes."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Optional, Callable
from vyping import Never

from vistutils.fields import AbstractField
from vistutils.waitaminute import typeMsg


class SpecialField(AbstractField):
  """SpecialField provides a descriptor class pointing to instances of
  custom classes."""

  @staticmethod
  def _searchCallable(cls: type, name: str) -> Optional[Callable]:
    """Searches given type for callable of given name"""
    if hasattr(cls, name):
      callMeMaybe = getattr(cls, name)
      if callable(callMeMaybe):
        return callMeMaybe
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)

  @staticmethod
  def _unbind(callMeMaybe: Callable) -> Callable:
    """If the callable received is bound, its __func__ attribute is
    returned. Why is this necessary? """
    if not callable(callMeMaybe):
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    if hasattr(callMeMaybe, '__func__'):
      return SpecialField._unbind(getattr(callMeMaybe, '__func__'))
    return callMeMaybe

  def __init_subclass__(cls, **kwargs) -> None:
    """Marks the new class as a subclass"""
    name = '__create_instance__'
    subCreateInstance = SpecialField._unbind(getattr(cls, name))
    baseCreateInstance = SpecialField._unbind(getattr(cls, name))
    if subCreateInstance is baseCreateInstance:
      delattr(cls, name)

  def __getattr__(self, key: str) -> Never:
    """Raises NotImplementedError if key is __create_instance__ instead of
    Attribute error. """
    if key == '__create_instance__':
      raise NotImplementedError(key)
    raise AttributeError(key)

  def __prepare_owner__(self, owner: type) -> type:
    """Subclasses must implement this method"""
    creatorName = self._getCreatorName()
    setattr(owner, creatorName, self._getInstanceCreator())
    return owner

  def __init__(self, fieldType: type, *args, **kwargs) -> None:
    if not isinstance(fieldType, type):
      e = typeMsg('fieldType', fieldType, type)
      raise TypeError(e)
    self.__field_type__ = fieldType
    AbstractField.__init__(self, *args, **kwargs)

  def _getFieldType(self) -> type:
    """Getter-function for field type"""
    if self.__field_type__ is not None:
      if isinstance(self.__field_type__, type):
        return self.__field_type__
      e = typeMsg('__field_type__', self.__field_type__, type)
      raise TypeError(e)
    raise RuntimeError

  def _getCreatorName(self) -> str:
    """Getter-function for the name of the creator function"""
    return '_create%s' % self._getCapName()

  def __create_instance__(self, ) -> Any:
    """This method is responsible for creating an instance of the field
    type. If the field type implements a method named __create_instance__,
    that method is called. Otherwise, a generic call to the __new__ method
    on the field type will be attempted.

    Subclasses of special field can reimplement this method.
    Alternatively, owners of a special field can use a decorator described
    below to specify a particular method for this instance. """

  def _getInstanceCreator(self) -> Callable:
    """Getter-function for creator function"""
    try:
      return self.__create_instance__
    except NotImplementedError:
      fieldType, fieldOwner = self._getFieldType(), self._getFieldOwner()
      types = [fieldType, fieldOwner]
      names = ['__create_instance__', self._getCreatorName()]
      for (type_, name) in zip(types, names):
        callMeMaybe = self._searchCallable(type_, name)
        if callMeMaybe is not None:
          return callMeMaybe
      if issubclass(fieldType, object):
        return object.__new__

  def CREATE(self, createFunction: Callable) -> Callable:
    """This decorator returns the function unaltered, so it should be
    directly above the function definition below any other decorator. """
    setattr(self, '__create_instance__', createFunction)
    return createFunction

  def _createInstance(self, instance: Any, owner: type) -> None:
    """Creates instance"""
    return self._getInstanceCreator()(instance, owner)

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    """Getter-function first attempts to locate an existing instance of
    field type on instance. Next, it attempts to create it. """
    if instance is None:
      return self.__get__(owner, owner, **kwargs)
    fieldName = self._getFieldName()
    pvtName = self._getPrivateName()
    fieldType = self._getFieldType()
    if hasattr(instance, pvtName):
      value = getattr(instance, pvtName)
      if isinstance(value, fieldType):
        return value
      e = typeMsg(fieldName, value, fieldType)
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    setattr(instance, pvtName, self._createInstance(instance, owner))
    return self.__get__(instance, owner, _recursion=True, **kwargs)
