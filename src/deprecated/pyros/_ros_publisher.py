"""RosPublisher encapsulates ros publisher logic"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from pyros import RosTopic, RosReg


class RosPublisher(RosTopic):
  """RosPublisher encapsulates ros publisher logic"""

  def _getRegType(self, ) -> RosReg:
    """Getter-function for the registration type"""
    return RosReg.PUB

  def _getDataClass(self) -> Any:
    """Getter-function for data class"""

  def _setDataClass(self, dataCls: Any) -> None:
    """Setter-function for data class"""
    self.__data_class__ = dataCls
