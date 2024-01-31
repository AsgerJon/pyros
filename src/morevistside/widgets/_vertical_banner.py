"""VerticalBanner provides a filled widget of fixed width"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic

from morevistside.widgets import FillWidget

ic.configureOutput(includeContext=True)


class VerticalBanner(FillWidget):
  """VerticalBanner provides a filled widget of fixed width"""

  def __init__(self, *args, **kwargs) -> None:
    FillWidget.__init__(self, *args, **kwargs)
    width = None
    for arg in args:
      if isinstance(arg, int) and width is None:
        height = arg
    ic(width)
    self.setFixedWidth(width)
