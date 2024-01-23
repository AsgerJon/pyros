"""Accessor is a cat class defining accessor types"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from morevistutils import Cat
from morevistutils.fields import QuickField


class Auto:
  """Placeholder thing lmao"""

  def __matmul__(self, other) -> Any:
    pass


auto = Auto


class Acc(Cat):
  """Accessor is a cat class defining accessor types"""

  GET: auto
  SET: auto
  DEL: auto

  def _getFieldAccessName(self) -> str:
    """Getter-function for the name of the associated operation in the
    field"""
    name = getattr(self, '__cat_name__')
    if name.lower() == 'get':
      return '__get__'
    if name.lower() == 'set':
      return '__set__'
    if name.lower() == 'del':
      return '__delete__'

  def __matmul__(self, other: QuickField):
    if other.__class__.__qualname__ != 'QuickField':
      raise NotImplemented

    def decorate(callMeMaybe: Callable) -> Callable:
      """Applies the decoration"""

      def wrapper(instance: Any, owner: type) -> Any:
        """Wraps the decorated function"""
        if instance is None:
          instance = owner
        try:
          return callMeMaybe(instance, owner)
        except TypeError as typeError:
          if 'argument' in str(typeError):
            try:
              return callMeMaybe(instance)
            except Exception as e:
              raise typeError from e

      setattr(other, self._getFieldAccessName(), wrapper)

      return callMeMaybe

    return decorate
