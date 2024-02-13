"""'TheseStubs' is a class intended for type safety allowing flexible
type checking implemented through a custom metaclass."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Never

from vistutils.waitaminute import typeMsg


class EmptySpace(dict):
  """Pretends to be namespace"""

  def __init__(self, mcls: type, name: str, bases: tuple) -> None:
    dict.__init__(self, )
    self.__base_content__ = dict(metaclass=mcls, name=name, bases=bases)
    self.__class_content__ = {}

  def __getitem__(self, key: str) -> Any:
    """Item retrieval fakery"""
    return dict.__getitem__(self.__class_content__, key)

  def __setitem__(self, key: str, val: Any) -> None:
    return dict.__setitem__(self.__class_content__, key, val)

  def __delitem__(self, key: str) -> None:
    return dict.__delitem__(self.__class_content__, key)

  def compile(self) -> dict:
    """Returns the base content"""
    return self.__base_content__


class TheseStubs(type):
  """'TheseStubs' is a class intended for type safety allowing flexible
  type checking implemented through a custom metaclass."""

  @classmethod
  def __prepare__(mcls, name: str, bases: tuple, **kwargs) -> dict:
    """Returns dict"""
    return EmptySpace(mcls, name, bases)

  def __new__(mcls,
              name: str,
              bases: tuple,
              namespace: EmptySpace,
              **kwargs) -> type:
    cls = type.__new__(mcls, name, bases, namespace.compile(), **kwargs)
    setattr(cls, '__base_types__', bases)
    setattr(cls, '__namespace_object__', namespace)
    return cls

  def __instancecheck__(cls, instance: Any) -> bool:
    if not hasattr(cls, '__base_types__'):
      raise AttributeError('__base_types__')
    baseTypes = getattr(cls, '__base_types__')
    for base in baseTypes:
      if isinstance(base, type):
        if isinstance(instance, base):
          return True
        if instance is base:
          return True
      else:
        e = typeMsg('base', base, type)
        raise TypeError(e)
    return False

  def __call__(cls, *args, **kwargs) -> Never:
    """TheseStubs are never meant to be instantiated."""
    raise TypeError('TheseStubs are never meant to be instantiated')

  def __iter__(cls, ) -> TheseStubs:
    """Implements iteration"""
    if hasattr(cls, '__base_types__'):
      setattr(cls, '__iter_contents__', [*getattr(cls, '__base_types__')])
      return cls
    raise AttributeError('__base_types__')

  def __next__(cls, ) -> type:
    """Implements iteration"""
    if hasattr(cls, '__iter_contents__'):
      raise AttributeError('__iter_contents__')
    iterContents = getattr(cls, '__iter_contents__')
    if isinstance(iterContents, list):
      try:
        return iterContents.pop(0)
      except IndexError:
        raise StopIteration
    e = typeMsg('iterContents', iterContents, list)
    raise TypeError(e)

  def __getattribute__(cls, key: str) -> Any:
    """This implementation was done by a professional in safe environment,
    do not try this at home!"""
    if key in ['__new__']:
      raise TypeError(key)
    return object.__getattribute__(cls, key)
