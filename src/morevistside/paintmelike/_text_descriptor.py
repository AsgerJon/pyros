"""The text descriptor provides a string valued descriptor. Subclasses can
then define printing. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from icecream import ic
from vistutils import maybe, monoSpace

from morevistside.paintmelike import AbstractStencil
from morevistutils import maybeType
from morevistutils.waitaminute import typeMsg


class TextDescriptor(AbstractStencil):
  """The text descriptor provides a string valued descriptor. Subclasses can
  then define printing. """

  __default_value__ = 'hello world!'

  def __init__(self, *args, **kwargs) -> None:
    AbstractStencil.__init__(self, *args, **kwargs)
    self.__default_value__ = maybeType(str, *args)

  def __get_default_value__(self, owner: type = None) -> str:
    """Getter-function for the default value"""
    if owner is None:
      return getattr(self, '__default_value__', None)
    ownerDefault = getattr(owner, '__default_value__', None)
    descriptorDefault = getattr(self, '__default_value__', None)
    defVal = maybe(ownerDefault, descriptorDefault)
    if isinstance(defVal, str):
      return defVal
    e = typeMsg('defVal', defVal, str)
    raise TypeError(e)

  def __get__(self, instance: Any, owner: type) -> str:
    """Implementation of getter function."""
    pvtName = self._getPrivateName()
    if instance is None:
      return self.__get_default_value__(owner)
    if hasattr(instance, pvtName):
      text = getattr(instance, pvtName)
      if isinstance(text, str):
        return text
      e = typeMsg('text', text, str)
      raise TypeError(e)
    defVal = self.__get_default_value__(owner)
    setattr(instance, pvtName, defVal)
    return defVal

  def __set__(self, instance: Any, value: str) -> None:
    """Implementation of setter function"""
    if not isinstance(value, str):
      e = typeMsg('value', value, str)
      raise TypeError(e)
    pvtName = self._getPrivateName()
    setattr(instance, pvtName, value)

  def __delete__(self, instance: Any) -> None:
    """Implementation of deleter function"""
    pvtName = self._getPrivateName()
    if hasattr(instance, pvtName):
      return setattr(instance, pvtName, '')
    e = """Tried to delete '%s' from '%s', which has no such attribute!"""
    raise AttributeError(monoSpace(e % (pvtName, instance)))
