"""The FixedSizeWidget implements the paint event to ensure that it takes
uyp the indicated amount of space. Byt default , this space is filled
transparent color, but this can be customized by passing anm instance of
QColor to the constructor. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QSize
from icecream import ic

from morevistside.widgets import FillWidget


class CornerWidget(FillWidget):
  """The FixedSizeWidget implements the paint event to ensure that it takes
  uyp the indicated amount of space. Byt default , this space is filled
  transparent color, but this can be customized by passing anm instance of
  QColor to the constructor. """

  def __init__(self, *args, **kwargs) -> None:
    FillWidget.__init__(self, *args, **kwargs)
    width, height = None, None
    for arg in args:
      if isinstance(arg, int):
        if width is None:
          width = arg
        elif height is None:
          height = arg
        else:
          break
    if width is None:
      width, height = 32, 32
    if height is None:
      height = width
    ic(width, height)
    size = QSize(width, height)
    self.setFixedSize(size)
