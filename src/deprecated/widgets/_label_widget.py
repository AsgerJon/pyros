"""LabelWidget provides a text containing widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtGui import QColor
from icecream import ic

from morevistside import parseHex
from morevistside.paintmelike import SolidBackground, TextLabel
from deprecated.widgets import PaintWidget
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class LabelWidget(PaintWidget):
  """LabelWidget provides a text containing widget"""

  fill = SolidBackground('#D0D0D0FF')
  text = TextLabel('LOL', 'Cambria', 32)

  def __init__(self, *args, **kwargs) -> None:
    PaintWidget.__init__(self, *args, **kwargs)
    defaultText = None
    defaultColor = None
    for arg in args:
      if isinstance(arg, str):
        if arg[0] == '#' and defaultColor is None:
          defaultColor = parseHex(arg)
        elif defaultText is None:
          defaultText = arg
      if isinstance(arg, QColor) and defaultColor is None:
        defaultColor = arg
    if defaultText is not None:
      if not isinstance(defaultText, str):
        e = typeMsg('defaultText', defaultText, str)
        raise TypeError(e)
      self.text = defaultText
    if defaultColor is not None:
      if not isinstance(defaultColor, QColor):
        e = typeMsg('defaultText', defaultText, QColor)
        raise TypeError(e)
      self.fill = defaultColor

  def textCallback(self, data: Any) -> None:
    """Callback test"""
    self.text = '%.3E' % data.data
    self.update()
