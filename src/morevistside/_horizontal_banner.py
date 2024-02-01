"""HorizontalBanner provides a filled widget of fixed height. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from icecream import ic

from morevistside.widgets import FillWidget

ic.configureOutput(includeContext=True)


class HorizontalBanner(FillWidget):
  """HorizontalBanner provides a filled widget of fixed height. """

  def __init__(self, *args, **kwargs) -> None:
    FillWidget.__init__(self, *args, **kwargs)
    height = None
    for arg in args:
      if isinstance(arg, int) and height is None:
        height = arg
    ic(height)
    self.setFixedHeight(height)
