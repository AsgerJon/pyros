"""EnvField implements access to environment variables through descriptor
classes. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
import os
from typing import Any

from vistutils.fields import AbstractField


class EnvField(AbstractField):
  """EnvField implements access to environment variables through descriptor
  classes. """

  def __prepare_owner__(self, owner: type) -> type:
    """Implementation of abstract method"""
    return owner

  def __init__(self, key: str, *args, **kwargs) -> None:
    self._key = key
    AbstractField.__init__(self, *args, **kwargs)

  def __get__(self, instance: Any, owner: type) -> Any:
    """Getter-function"""
    if self._key in os.environ:
      return os.environ[self._key]
    raise KeyError(self._key)

  def __set__(self, instance: Any, value: Any) -> None:
    """Setter-function"""
    os.environ[self._key] = value

  def __delete__(self, instance: Any) -> None:
    """Deleter-function"""
    if self._key in os.environ:
      del os.environ[self._key]
    else:
      raise KeyError(self._key)
