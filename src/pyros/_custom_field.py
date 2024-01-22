"""The CustomField class provides descriptor class, where accessor methods
must be decorated as such and defined on the owning class itself."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from vistutils import monoSpace, stringList
from vistutils.fields import AbstractField


class CustomField(AbstractField):
  """The CustomField class provides descriptor class, where accessor methods
  must be decorated as such and defined on the owning class itself."""

  def __init__(self, *args, **kwargs) -> None:
    AbstractField.__init__(self, *args, **kwargs, )

  def __prepare_owner__(self, owner: type) -> type:
    """Prepares owner"""
    return owner

  def _getDecoratedCallables(self, owner: type) -> list[dict[str, Callable]]:
    """Getter-function for list of decorated callables found in the name
    space of the class received."""
    out = []
    for (key, val) in owner.__dict__.items():
      if hasattr(val, '__is_decorated__', ) and callable(val):
        if hasattr(val, self.__field_name__):
          accessor = getattr(val, self.__field_name__)
          out.append(dict(accessor=accessor, function=val))
    return out

  def _getAccessor(self, owner: type, accessor: str) -> Callable:
    """Getter-function for the accessor method specified."""
    for (access, callMeMaybe) in self._getDecoratedCallables(owner):
      if access == accessor:
        return callMeMaybe
    e = """Found no accessor function of access type: '%s' for field: '%s'
    on owner: '%s'!"""
    fieldName = self.__field_name__
    ownerName = self.__field_owner__.__qualname__
    msg = monoSpace(e % (accessor, fieldName, ownerName))
    return lambda *__, **_: self
    raise AttributeError(msg)

  def __get__(self, instance: Any, owner: type) -> Any:
    """Getter implementation"""
    if instance is None:
      return self.__get__(owner, owner)
    getter = self._getAccessor(owner, 'get')
    if hasattr(getter, '__self__'):
      return getter()
    return getter(instance)

  def __set__(self, instance: Any, value: Any) -> None:
    """Setter-implementation"""
    owner = self.__field_owner__
    setter = self._getAccessor(owner, 'set')
    if hasattr(setter, '__self__'):
      return setter(value)
    return setter(instance, value)

  def __delete__(self, instance: Any) -> None:
    """Deleter implementation"""
    owner = self.__field_owner__
    deleter = self._getAccessor(owner, 'delete')
    if hasattr(deleter, '__self__'):
      return deleter()
    return deleter(instance)
