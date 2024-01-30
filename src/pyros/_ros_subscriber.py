"""The RosSubscriber encapsulates the ros subscriber logic"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from pyros import RosTopic, RosReg
import std_msgs.msg as msg


class RosSubscriber(RosTopic):
  """The RosSubscriber encapsulates the ros subscriber logic"""

  def _getRegType(self, ) -> RosReg:
    """Getter-function for the registration type"""
    return RosReg.SUB

  def _getDataClass(self) -> Any:
    """Getter-function for data class"""

  def _setDataClass(self, dataCls: Any) -> None:
    """Setter-function for data class"""
    self.__data_class__ = dataCls
