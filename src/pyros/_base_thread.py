"""BaseThread is mostly a test"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any

from PySide6.QtCore import Signal

from pyros import AbstractRosThread, RosField


class BaseThread(AbstractRosThread):
  """BaseThread is mostly a test"""

  pumpCurrent = RosField('/pump/current', )
  pumpSignal = Signal(float, float)

  @pumpCurrent.CALL
  def pumpCurrent(self, data: Any) -> None:
    """Set the current of the pump."""
    value = data.data
    self.pumpSignal.emit(time.time(), value)
