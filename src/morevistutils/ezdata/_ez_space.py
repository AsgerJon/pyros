"""The EZSpace provides the namespace object used by the EZMeta metaclass
to create the EZData class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from morevistutils.metas import BaseNamespace


class EZSpace(BaseNamespace):
  """The EZSpace provides the namespace object used by the EZMeta metaclass
  to create the EZData class."""

  @staticmethod
  def _getGlobals() -> dict:
    """Getter-function for globals"""
    return {**globals(), }

  def getCallables(self) -> dict:
    """Getter-function for the dictionary containing the functions"""
    out = {}
    for (key, val) in dict.items(self, ):
      if callable(val):
        out |= {key: val}
    return out

  @classmethod
  def resolveType(cls, typeName: str) -> type:
    """Resolves the name to the type"""
    type_ = cls._getGlobals().get(typeName, None)
    if type_ is None:
      raise NameError(typeName)
    if isinstance(type_, type):
      return type_
    raise TypeError(typeName)

  def compile(self) -> dict:
    """Collects annotations and callables"""
    out = {}
    for (key, val) in self.getAnnotations().items():
      out |= {key: self.resolveType(val)}
    return {**out, **self.getCallables()}
