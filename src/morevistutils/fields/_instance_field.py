"""InstanceField wraps a class and instantiates it for every different
instance calling the __get__. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from morevistutils.waitaminute import typeMsg


class InstanceField:
  """InstanceField wraps a class and instantiates it for every different
  instance calling the __get__. """

  def __init__(self, cls: type, *args, **kwargs) -> None:
    if not isinstance(cls, type):
      e = typeMsg('cls', cls, type)
      raise TypeError(e)
    self.__inner_class__ = cls
    self.__field_name__ = None
    self.__field_owner__ = None
    self.__positional_arguments__ = args
    self.__keyword_arguments__ = kwargs

  def __set_name__(self, owner: type, name: str) -> None:
    """Invoked automatically when InstanceField is instantiated in a class
    body."""
    self.__field_owner__ = owner
    self.__field_name__ = name

  def _getFieldName(self) -> str:
    """Getter-function for field name"""
    return self.__field_name__

  def _getFieldOwner(self) -> type:
    """Getter-function for the field owner"""
    return self.__field_owner__

  def _getInnerClass(self) -> type:
    """Getter-function for the inner class"""
    return self.__inner_class__

  def _getPrivateVariableName(self) -> str:
    """Getter-function for the private name at which the dedicated
    instance is created."""
    ownerName = self._getFieldOwner().__qualname__
    fieldName = self._getFieldName()
    innerName = self._getInnerClass().__qualname__
    return '__%s_%s_%s_var__' % (ownerName, fieldName, innerName)

  def _getPrivateSetterName(self) -> str:
    """Getter-function for the optional setter method"""
    return self._getPrivateVariableName().replace('var', 'set')

  def _createInnerInstance(self, instance: Any, ) -> None:
    """Creator function inner class"""
    innerClass = self._getInnerClass()
    args = self.__positional_arguments__
    kwargs = self.__keyword_arguments__
    innerInstance = innerClass(instance, *args, **kwargs)
    pvtName = self._getPrivateVariableName()
    setattr(instance, pvtName, innerInstance)

  def __get__(self, instance: Any, owner: type, **kwargs) -> Any:
    """Getter-function returns a dedicated instance of inner class. If
    this does not already exist, it is created."""
    if instance is None:
      return self._getInnerClass()
    pvtName = self._getPrivateVariableName()
    if hasattr(instance, pvtName):
      return getattr(instance, pvtName)
    if kwargs.get('recursion', False):
      raise RecursionError
    self._createInnerInstance(instance)
    return self.__get__(instance, owner, _recursion=True, **kwargs)

  def __set__(self, instance: Any, value: Any) -> None:
    """Must be defined in the class body with the SET decorator."""
    setter = self._getSetter()
    return setter(instance, value)

  def SET(self, callMeMaybe: Callable) -> Callable:
    """Decorator setting received callable as the setter function for
    this. Recall that decorators received the unbounded function, meaning
    that ...(self, value: Any) is the signature expected, rather than:
    ...(self, instance: Any, value: Any).
    Please note that this decorator returns the callable as it was
    received. For this reason, it should be the decorator applies first
    meaning that it should appear below other decorators, for example:
    @abstractmethod
    @someField.SET
    def setSomeField(self, value: Any) -> None:
      ... """
    pvtName = self._getPrivateSetterName()
    owner = self._getFieldOwner()
    setattr(owner, pvtName, callMeMaybe)
    return callMeMaybe

  def _getSetter(self, ) -> Callable:
    """Getter-function for the setter method on the owner class. Please
    note that if no setter is explicitly defined in the owner with the SET
    decorator defined above, TypeError explaining read-only will be
    raised 'from' the expected AttributeError."""
    owner = self._getFieldOwner()
    pvtName = self._getPrivateSetterName()
    if hasattr(owner, pvtName):
      setter = getattr(owner, pvtName)
      if callable(setter):
        return setter
      e = typeMsg('setter', setter, Callable)
      raise TypeError
    try:
      return getattr(owner, pvtName)
    except AttributeError as attributeError:
      fieldName = self._getFieldName()
      fieldOwner = self._getFieldOwner()
      e = """The field '%s' on class '%s' is read-only!"""
      msg = e % (fieldName, fieldOwner)
      raise TypeError(msg) from attributeError
