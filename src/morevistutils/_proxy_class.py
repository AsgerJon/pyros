"""ProxyClass provides a temporary stand-in for a real class not yet
ready."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any


class ProxyClass(type):
  """ProxyClass provides a temporary stand-in for a real class not yet
  ready."""

  def _getNames(cls) -> list:
    """Getter-function for list of names"""
    return [cls.__name__, cls.__qualname__]

  def __new__(mcls, name: str, *__, **_, ) -> Any:
    return type.__new__(mcls, name, (), {})

  def __instancecheck__(cls, self: Any) -> bool:
    return True if self.__class__.__qualname__ == cls._getNames() else False
