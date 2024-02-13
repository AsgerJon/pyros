"""DataWidget provides a widget implementation representing data as points
on the viewport. The data should present both a real representation
pertaining the real values as well as in terms of the pixels available for
painting. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPaintEvent
from vistutils.fields import Field

from morevistside.widgets import FillWidget


class DataWidget(FillWidget):
  """DataWidget provides a widget implementation representing data as points
  on the viewport. The data should present both a real representation
  pertaining the real values as well as in terms of the pixels available for
  painting. """

  dataPen = Field()

  def __init__(self, *args, **kwargs) -> None:
    FillWidget.__init__(self, *args, **kwargs)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paint implementation"""
