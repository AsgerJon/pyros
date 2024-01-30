"""The EZMeta is the metaclass used by the EZData class ensuring the
dataclass behaviour. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils.metas import AbstractMetaclass, Bases

from morevistutils.ezdata import EZSpace


class EZMeta(AbstractMetaclass):
  """The EZMeta is the metaclass used by the EZData class ensuring the
  dataclass behaviour. """

  @classmethod
  def __prepare__(mcls, name: str, bases: Bases, **kwargs) -> EZSpace:
    """Creates the namespace object"""
    raise NotImplementedError
