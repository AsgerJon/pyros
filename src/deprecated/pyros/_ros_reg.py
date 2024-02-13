"""RosReg provides an enum class for different Registration types."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from enum import Enum


class RosReg(Enum):
  """RosReg provides an enum class for different Registration types."""

  PUB = 0
  SUB = 1
  SRV = 2

  def __str__(self, ) -> str:
    """String representation"""
    return '%s' % self.name.lower()
