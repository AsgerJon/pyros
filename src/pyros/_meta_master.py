"""MetaMaster provides a metaclass specifically for the creation of teh
RosMaster class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen

from __future__ import annotations

from utils.metas import Bases


class MetaMaster(type):
  """MetaMaster provides a metaclass specifically for the creation of
  the RosMaster class"""

  @classmethod
  def __prepare__(mcls, name: str, bases: Bases, **kwargs) -> dict[
    str, type]:
    return {}
